// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (c) 2022-2024 Helge Deller <deller@gmx.de>
 *
 *  based on arch/s390/kernel/vdso.c which is
 *  Copyright IBM Corp. 2008
 *  Author(s): Martin Schwidefsky (schwidefsky@de.ibm.com)
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/elf.h>
#include <linux/timekeeper_internal.h>
#include <linux/compat.h>
#include <linux/nsproxy.h>
#include <linux/time_namespace.h>
#include <linux/random.h>

#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/sections.h>
#include <asm/vdso.h>
#include <asm/cacheflush.h>

#include <vdso/datapage.h>

extern char vdso32_start, vdso32_end;
extern char vdso64_start, vdso64_end;

static struct vm_special_mapping vvar_mapping;

static union vdso_data_store vdso_data_store __page_aligned_data;

struct vdso_data *vdso_data = vdso_data_store.data;

enum vvar_pages {
	VVAR_DATA_PAGE_OFFSET,
	VVAR_TIMENS_PAGE_OFFSET,
	VVAR_NR_PAGES,
};

#ifdef CONFIG_TIME_NS
struct vdso_data *arch_get_vdso_data(void *vvar_page)
{
	return (struct vdso_data *)(vvar_page);
}

/*
 * The VVAR page layout depends on whether a task belongs to the root or
 * non-root time namespace. Whenever a task changes its namespace, the VVAR
 * page tables are cleared and then they will be re-faulted with a
 * corresponding layout.
 * See also the comment near timens_setup_vdso_data() for details.
 */
int vdso_join_timens(struct task_struct *task, struct time_namespace *ns)
{
	struct mm_struct *mm = task->mm;
	VMA_ITERATOR(vmi, mm, 0);
	struct vm_area_struct *vma;

	mmap_read_lock(mm);
	for_each_vma(vmi, vma) {
		if (!vma_is_special_mapping(vma, &vvar_mapping))
			continue;
		zap_vma_pages(vma);
		break;
	}
	mmap_read_unlock(mm);
	return 0;
}
#endif

static vm_fault_t vvar_fault(const struct vm_special_mapping *sm,
			     struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct page *timens_page = find_timens_vvar_page(vma);
	unsigned long addr, pfn;
	vm_fault_t err;

	switch (vmf->pgoff) {
	case VVAR_DATA_PAGE_OFFSET:
		pfn = virt_to_pfn(vdso_data);
		if (timens_page) {
			/*
			 * Fault in VVAR page too, since it will be accessed
			 * to get clock data anyway.
			 */
			addr = vmf->address + VVAR_TIMENS_PAGE_OFFSET * PAGE_SIZE;
			err = vmf_insert_pfn(vma, addr, pfn);
			if (unlikely(err & VM_FAULT_ERROR))
				return err;
			pfn = page_to_pfn(timens_page);
		}
		break;
#ifdef CONFIG_TIME_NS
	case VVAR_TIMENS_PAGE_OFFSET:
		/*
		 * If a task belongs to a time namespace then a namespace
		 * specific VVAR is mapped with the VVAR_DATA_PAGE_OFFSET and
		 * the real VVAR page is mapped with the VVAR_TIMENS_PAGE_OFFSET
		 * offset.
		 * See also the comment near timens_setup_vdso_data().
		 */
		if (!timens_page)
			return VM_FAULT_SIGBUS;
		pfn = virt_to_pfn(vdso_data);
		break;
#endif /* CONFIG_TIME_NS */
	default:
		return VM_FAULT_SIGBUS;
	}
	return vmf_insert_pfn(vma, vmf->address, pfn);
}

static int vdso_mremap(const struct vm_special_mapping *sm,
		       struct vm_area_struct *vma)
{
	current->mm->context.vdso_base = vma->vm_start;
	return 0;
}

static struct vm_special_mapping vvar_mapping = {
	.name = "[vvar]",
	.fault = vvar_fault,
};

#ifdef CONFIG_64BIT
static struct vm_special_mapping vdso64_mapping = {
	.name = "[vdso]",
	.mremap = vdso_mremap,
};
#endif

static struct vm_special_mapping vdso32_mapping = {
	.name = "[vdso]",
	.mremap = vdso_mremap,
};

/*
 * This is called from binfmt_elf, we create the special vma for the
 * vDSO and insert it into the mm struct tree
 */
int arch_setup_additional_pages(struct linux_binprm *bprm,
				int executable_stack)
{

	unsigned long vdso_text_start, vdso_text_len, map_base;
	unsigned long vvar_start, vdso_mapping_len;
	struct vm_special_mapping *vdso_mapping;
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;
	int rc;

	if (mmap_write_lock_killable(mm))
		return -EINTR;

#ifdef CONFIG_64BIT
	if (!is_compat_task()) {
		vdso_text_len = &vdso64_end - &vdso64_start;
		vdso_mapping = &vdso64_mapping;
	} else
#endif
	{
		vdso_text_len = &vdso32_end - &vdso32_start;
		vdso_mapping = &vdso32_mapping;
	}

	map_base = mm->mmap_base;
	if (current->flags & PF_RANDOMIZE)
		map_base -= get_random_u32_below(0x20) * PAGE_SIZE;

	vdso_mapping_len = vdso_text_len + VVAR_NR_PAGES * PAGE_SIZE;
	vvar_start = get_unmapped_area(NULL, map_base, vdso_mapping_len, 0, 0);
	rc = vvar_start;
	if (IS_ERR_VALUE(vvar_start))
		goto out;
	BUILD_BUG_ON(VVAR_NR_PAGES != __VVAR_PAGES);
	vma = _install_special_mapping(mm, vvar_start, VVAR_NR_PAGES*PAGE_SIZE,
				       VM_READ|VM_MAYREAD|VM_IO|VM_DONTDUMP|
				       VM_PFNMAP,
				       &vvar_mapping);
	rc = PTR_ERR(vma);
	if (IS_ERR(vma))
		goto out;
	vdso_text_start = vvar_start + VVAR_NR_PAGES * PAGE_SIZE;

	/* VM_MAYWRITE for COW so gdb can set breakpoints */
	vma = _install_special_mapping(mm, vdso_text_start, vdso_text_len,
				       VM_READ|VM_EXEC|
				       VM_MAYREAD|VM_MAYWRITE|VM_MAYEXEC,
				       vdso_mapping);
	if (IS_ERR(vma)) {
		do_munmap(mm, vdso_text_start, PAGE_SIZE, NULL);
		rc = PTR_ERR(vma);
	} else {
		current->mm->context.vdso_base = vdso_text_start;
		rc = 0;
	}
out:
	mmap_write_unlock(mm);
	return rc;
}

static struct page ** __init vdso_setup_pages(void *start, void *end)
{
	int pages = (end - start) >> PAGE_SHIFT;
	struct page **pagelist;
	int i;

	pagelist = kcalloc(pages + 1, sizeof(struct page *), GFP_KERNEL);
	if (!pagelist)
		panic("%s: Cannot allocate page list for VDSO", __func__);
	for (i = 0; i < pages; i++)
		pagelist[i] = virt_to_page(start + i * PAGE_SIZE);
	return pagelist;
}

static int __init vdso_init(void)
{
#ifdef CONFIG_64BIT
	vdso64_mapping.pages = vdso_setup_pages(&vdso64_start, &vdso64_end);
#endif
	if (IS_ENABLED(CONFIG_COMPAT) || !IS_ENABLED(CONFIG_64BIT))
		vdso32_mapping.pages = vdso_setup_pages(&vdso32_start, &vdso32_end);
	return 0;
}
arch_initcall(vdso_init);
