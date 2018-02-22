/*
 * Copyright (C) 2010-2012 ARM Limited. All rights reserved.
 * 
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU licence.
 * 
 * A copy of the licence is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <linux/version.h>

#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
#include <linux/shrinker.h>
#endif

#include "mali_osk.h"
#include "mali_ukk.h" 
#include "mali_kernel_common.h"
#include "mali_kernel_linux.h"

static void mali_kernel_memory_vma_open(struct vm_area_struct * vma);
static void mali_kernel_memory_vma_close(struct vm_area_struct * vma);


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
static int mali_kernel_memory_cpu_page_fault_handler(struct vm_area_struct *vma, struct vm_fault *vmf);
#else
static unsigned long mali_kernel_memory_cpu_page_fault_handler(struct vm_area_struct * vma, unsigned long address);
#endif


typedef struct mali_vma_usage_tracker
{
	int references;
	u32 cookie;
} mali_vma_usage_tracker;


struct AllocationList
{
	struct AllocationList *next;
	u32 offset;
	u32 physaddr;
};

typedef struct AllocationList AllocationList;

struct MappingInfo
{
	struct vm_area_struct *vma;
	struct AllocationList *list;
	struct AllocationList *tail;
};

typedef struct MappingInfo MappingInfo;


static u32 _kernel_page_allocate(void);
static void _kernel_page_release(u32 physical_address);
static AllocationList * _allocation_list_item_get(void);
static void _allocation_list_item_release(AllocationList * item);


static DEFINE_SPINLOCK(allocation_list_spinlock);
static AllocationList * pre_allocated_memory = (AllocationList*) NULL ;
static int pre_allocated_memory_size_current  = 0;
#ifdef MALI_OS_MEMORY_KERNEL_BUFFER_SIZE_IN_MB
	static int pre_allocated_memory_size_max      = MALI_OS_MEMORY_KERNEL_BUFFER_SIZE_IN_MB * 1024 * 1024;
#else
	static int pre_allocated_memory_size_max      = 16 * 1024 * 1024; 
#endif

static struct vm_operations_struct mali_kernel_vm_ops =
{
	.open = mali_kernel_memory_vma_open,
	.close = mali_kernel_memory_vma_close,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	.fault = mali_kernel_memory_cpu_page_fault_handler
#else
	.nopfn = mali_kernel_memory_cpu_page_fault_handler
#endif
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static int mali_mem_shrink(int nr_to_scan, gfp_t gfp_mask)
	#else
static int mali_mem_shrink(struct shrinker *shrinker, int nr_to_scan, gfp_t gfp_mask)
	#endif
#else
static int mali_mem_shrink(struct shrinker *shrinker, struct shrink_control *sc)
#endif
{
	unsigned long flags;
	AllocationList *item;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
	int nr = nr_to_scan;
#else
	int nr = sc->nr_to_scan;
#endif

	if (0 == nr)
	{
		return pre_allocated_memory_size_current / PAGE_SIZE;
	}

	if (0 == pre_allocated_memory_size_current)
	{
		
		return 0;
	}

	if (0 == spin_trylock_irqsave(&allocation_list_spinlock, flags))
	{
		
		return -1;
	}

	while (pre_allocated_memory && nr > 0)
	{
		item = pre_allocated_memory;
		pre_allocated_memory = item->next;

		_kernel_page_release(item->physaddr);
		_mali_osk_free(item);

		pre_allocated_memory_size_current -= PAGE_SIZE;
		--nr;
	}
	spin_unlock_irqrestore(&allocation_list_spinlock,flags);

	return pre_allocated_memory_size_current / PAGE_SIZE;
}

struct shrinker mali_mem_shrinker = {
	.shrink = mali_mem_shrink,
	.seeks = DEFAULT_SEEKS,
};

void mali_osk_low_level_mem_init(void)
{
	pre_allocated_memory = (AllocationList*) NULL ;

	register_shrinker(&mali_mem_shrinker);
}

void mali_osk_low_level_mem_term(void)
{
	unregister_shrinker(&mali_mem_shrinker);

	while ( NULL != pre_allocated_memory )
	{
		AllocationList *item;
		item = pre_allocated_memory;
		pre_allocated_memory = item->next;
		_kernel_page_release(item->physaddr);
		_mali_osk_free( item );
	}
	pre_allocated_memory_size_current  = 0;
}

static u32 _kernel_page_allocate(void)
{
	struct page *new_page;
	u32 linux_phys_addr;

	new_page = alloc_page(GFP_HIGHUSER | __GFP_ZERO | __GFP_NOWARN | __GFP_COLD);

	if ( NULL == new_page )
	{
		return 0;
	}

	
	linux_phys_addr = dma_map_page(NULL, new_page, 0, PAGE_SIZE, DMA_BIDIRECTIONAL);

	return linux_phys_addr;
}

static void _kernel_page_release(u32 physical_address)
{
	struct page *unmap_page;

	#if 1
	dma_unmap_page(NULL, physical_address, PAGE_SIZE, DMA_BIDIRECTIONAL);
	#endif

	unmap_page = pfn_to_page( physical_address >> PAGE_SHIFT );
	MALI_DEBUG_ASSERT_POINTER( unmap_page );
	__free_page( unmap_page );
}

static AllocationList * _allocation_list_item_get(void)
{
	AllocationList *item = NULL;
	unsigned long flags;

	spin_lock_irqsave(&allocation_list_spinlock,flags);
	if ( pre_allocated_memory )
	{
		item = pre_allocated_memory;
		pre_allocated_memory = pre_allocated_memory->next;
		pre_allocated_memory_size_current -= PAGE_SIZE;

		spin_unlock_irqrestore(&allocation_list_spinlock,flags);
		return item;
	}
	spin_unlock_irqrestore(&allocation_list_spinlock,flags);

	item = _mali_osk_malloc( sizeof(AllocationList) );
	if ( NULL == item)
	{
		return NULL;
	}

	item->physaddr = _kernel_page_allocate();
	if ( 0 == item->physaddr )
	{
		
		_mali_osk_free( item );
		return NULL;
	}
	return item;
}

static void _allocation_list_item_release(AllocationList * item)
{
	unsigned long flags;
	spin_lock_irqsave(&allocation_list_spinlock,flags);
	if ( pre_allocated_memory_size_current < pre_allocated_memory_size_max)
	{
		item->next = pre_allocated_memory;
		pre_allocated_memory = item;
		pre_allocated_memory_size_current += PAGE_SIZE;
		spin_unlock_irqrestore(&allocation_list_spinlock,flags);
		return;
	}
	spin_unlock_irqrestore(&allocation_list_spinlock,flags);

	_kernel_page_release(item->physaddr);
	_mali_osk_free( item );
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
static int mali_kernel_memory_cpu_page_fault_handler(struct vm_area_struct *vma, struct vm_fault *vmf)
#else
static unsigned long mali_kernel_memory_cpu_page_fault_handler(struct vm_area_struct * vma, unsigned long address)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	void __user * address;
	address = vmf->virtual_address;
#endif

	MALI_DEBUG_PRINT(1, ("Page-fault in Mali memory region caused by the CPU.\n"));
	MALI_DEBUG_PRINT(1, ("Tried to access %p (process local virtual address) which is not currently mapped to any Mali memory.\n", (void*)address));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	return VM_FAULT_SIGBUS;
#else
	return NOPFN_SIGBUS;
#endif
}

static void mali_kernel_memory_vma_open(struct vm_area_struct * vma)
{
	mali_vma_usage_tracker * vma_usage_tracker;
	MALI_DEBUG_PRINT(4, ("Open called on vma %p\n", vma));

	vma_usage_tracker = (mali_vma_usage_tracker*)vma->vm_private_data;
	vma_usage_tracker->references++;

	return;
}

static void mali_kernel_memory_vma_close(struct vm_area_struct * vma)
{
	_mali_uk_mem_munmap_s args = {0, };
	mali_memory_allocation * descriptor;
	mali_vma_usage_tracker * vma_usage_tracker;
	MALI_DEBUG_PRINT(3, ("Close called on vma %p\n", vma));

	vma_usage_tracker = (mali_vma_usage_tracker*)vma->vm_private_data;

	BUG_ON(!vma_usage_tracker);
	BUG_ON(0 == vma_usage_tracker->references);

	vma_usage_tracker->references--;

	if (0 != vma_usage_tracker->references)
	{
		MALI_DEBUG_PRINT(3, ("Ignoring this close, %d references still exists\n", vma_usage_tracker->references));
		return;
	}


	descriptor = (mali_memory_allocation *)vma_usage_tracker->cookie;

	args.cookie = (u32)descriptor;
	args.mapping = descriptor->mapping;
	args.size = descriptor->size;

	_mali_ukk_mem_munmap( &args );

}


void _mali_osk_mem_barrier( void )
{
	mb();
}

void _mali_osk_write_mem_barrier( void )
{
	wmb();
}

mali_io_address _mali_osk_mem_mapioregion( u32 phys, u32 size, const char *description )
{
	return (mali_io_address)ioremap_nocache(phys, size);
}

void _mali_osk_mem_unmapioregion( u32 phys, u32 size, mali_io_address virt )
{
	iounmap((void*)virt);
}

mali_io_address _mali_osk_mem_allocioregion( u32 *phys, u32 size )
{
	void * virt;
 	MALI_DEBUG_ASSERT_POINTER( phys );
 	MALI_DEBUG_ASSERT( 0 == (size & ~_MALI_OSK_CPU_PAGE_MASK) );
 	MALI_DEBUG_ASSERT( 0 != size );

#ifdef SPRD_MEM_OPT_PAGE_TABLE_DEFRAGMENTIZE
	virt = dma_alloc_coherent(NULL, size, phys, GFP_NOWAIT | GFP_KERNEL | GFP_DMA );
#else
	virt = dma_alloc_coherent(NULL, size, phys, GFP_KERNEL | GFP_DMA );
#endif

	MALI_DEBUG_PRINT(3, ("Page table virt: 0x%x = dma_alloc_coherent(size:%d, phys:0x%x, )\n", virt, size, phys));

 	if ( NULL == virt )
 	{
		MALI_DEBUG_PRINT(5, ("allocioregion: Failed to allocate Pagetable memory, size=0x%.8X\n", size ));
 		return 0;
 	}

	MALI_DEBUG_ASSERT( 0 == (*phys & ~_MALI_OSK_CPU_PAGE_MASK) );

 	return (mali_io_address)virt;
}

void _mali_osk_mem_freeioregion( u32 phys, u32 size, mali_io_address virt )
{
 	MALI_DEBUG_ASSERT_POINTER( (void*)virt );
 	MALI_DEBUG_ASSERT( 0 != size );
 	MALI_DEBUG_ASSERT( 0 == (phys & ( (1 << PAGE_SHIFT) - 1 )) );

	dma_free_coherent(NULL, size, virt, phys);
}

_mali_osk_errcode_t inline _mali_osk_mem_reqregion( u32 phys, u32 size, const char *description )
{
	return ((NULL == request_mem_region(phys, size, description)) ? _MALI_OSK_ERR_NOMEM : _MALI_OSK_ERR_OK);
}

void inline _mali_osk_mem_unreqregion( u32 phys, u32 size )
{
	release_mem_region(phys, size);
}

void inline _mali_osk_mem_iowrite32_relaxed( volatile mali_io_address addr, u32 offset, u32 val )
{
	__raw_writel(cpu_to_le32(val),((u8*)addr) + offset);
}

u32 inline _mali_osk_mem_ioread32( volatile mali_io_address addr, u32 offset )
{
	return ioread32(((u8*)addr) + offset);
}

void inline _mali_osk_mem_iowrite32( volatile mali_io_address addr, u32 offset, u32 val )
{
	iowrite32(val, ((u8*)addr) + offset);
}

void _mali_osk_cache_flushall( void )
{
	
}

void _mali_osk_cache_ensure_uncached_range_flushed( void *uncached_mapping, u32 offset, u32 size )
{
	_mali_osk_write_mem_barrier();
}

_mali_osk_errcode_t _mali_osk_mem_mapregion_init( mali_memory_allocation * descriptor )
{
	struct vm_area_struct *vma;
	mali_vma_usage_tracker * vma_usage_tracker;
	MappingInfo *mappingInfo;

	if (NULL == descriptor) return _MALI_OSK_ERR_FAULT;

	MALI_DEBUG_ASSERT( 0 != (descriptor->flags & MALI_MEMORY_ALLOCATION_FLAG_MAP_INTO_USERSPACE) );

	vma = (struct vm_area_struct*)descriptor->process_addr_mapping_info;

	if (NULL == vma ) return _MALI_OSK_ERR_FAULT;

	
	mappingInfo = _mali_osk_calloc( 1, sizeof(MappingInfo) );

	if ( NULL == mappingInfo ) return _MALI_OSK_ERR_FAULT;

	vma_usage_tracker = _mali_osk_calloc( 1, sizeof(mali_vma_usage_tracker) );

	if (NULL == vma_usage_tracker)
	{
		MALI_DEBUG_PRINT(2, ("Failed to allocate memory to track memory usage\n"));
		_mali_osk_free( mappingInfo );
		return _MALI_OSK_ERR_FAULT;
	}

	mappingInfo->vma = vma;
	descriptor->process_addr_mapping_info = mappingInfo;

	
	descriptor->mapping = (void __user*)vma->vm_start;
	

	vma->vm_flags |= VM_IO;
	vma->vm_flags |= VM_RESERVED;
	vma->vm_flags |= VM_DONTCOPY;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_ops = &mali_kernel_vm_ops; 

	vma_usage_tracker->references = 1; 
	vma_usage_tracker->cookie = (u32)descriptor; 

	vma->vm_private_data = vma_usage_tracker;

	return _MALI_OSK_ERR_OK;
}

void _mali_osk_mem_mapregion_term( mali_memory_allocation * descriptor )
{
	struct vm_area_struct* vma;
	mali_vma_usage_tracker * vma_usage_tracker;
	MappingInfo *mappingInfo;

	if (NULL == descriptor) return;

	MALI_DEBUG_ASSERT( 0 != (descriptor->flags & MALI_MEMORY_ALLOCATION_FLAG_MAP_INTO_USERSPACE) );

	mappingInfo = (MappingInfo *)descriptor->process_addr_mapping_info;

	MALI_DEBUG_ASSERT_POINTER( mappingInfo );

	vma = mappingInfo->vma;

	MALI_DEBUG_ASSERT_POINTER( vma );


	vma_usage_tracker = vma->vm_private_data;

	
	_mali_osk_free(vma_usage_tracker);

	_mali_osk_free( mappingInfo );
	return;
}

_mali_osk_errcode_t _mali_osk_mem_mapregion_map( mali_memory_allocation * descriptor, u32 offset, u32 *phys_addr, u32 size )
{
	struct vm_area_struct *vma;
	MappingInfo *mappingInfo;

	if (NULL == descriptor) return _MALI_OSK_ERR_FAULT;

	MALI_DEBUG_ASSERT_POINTER( phys_addr );

	MALI_DEBUG_ASSERT( 0 != (descriptor->flags & MALI_MEMORY_ALLOCATION_FLAG_MAP_INTO_USERSPACE) );

	MALI_DEBUG_ASSERT( 0 == (size & ~_MALI_OSK_CPU_PAGE_MASK) );

	MALI_DEBUG_ASSERT( 0 == (offset & ~_MALI_OSK_CPU_PAGE_MASK));

	if (NULL == descriptor->mapping) return _MALI_OSK_ERR_INVALID_ARGS;

	if (size > (descriptor->size - offset))
	{
		MALI_DEBUG_PRINT(1,("_mali_osk_mem_mapregion_map: virtual memory area not large enough to map physical 0x%x size %x into area 0x%x at offset 0x%xr\n",
		                    *phys_addr, size, descriptor->mapping, offset));
		return _MALI_OSK_ERR_FAULT;
	}

	mappingInfo = (MappingInfo *)descriptor->process_addr_mapping_info;

	MALI_DEBUG_ASSERT_POINTER( mappingInfo );

	vma = mappingInfo->vma;

	if (NULL == vma ) return _MALI_OSK_ERR_FAULT;

	MALI_DEBUG_PRINT(7, ("Process map: mapping 0x%08X to process address 0x%08lX length 0x%08X\n", *phys_addr, (long unsigned int)(descriptor->mapping + offset), size));

	if ( MALI_MEMORY_ALLOCATION_OS_ALLOCATED_PHYSADDR_MAGIC == *phys_addr )
	{
		_mali_osk_errcode_t ret;
		AllocationList *alloc_item;
		u32 linux_phys_frame_num;

		alloc_item = _allocation_list_item_get();
		if (NULL == alloc_item)
		{
			MALI_DEBUG_PRINT(1, ("Failed to allocate list item\n"));
			return _MALI_OSK_ERR_NOMEM;
		}

		linux_phys_frame_num = alloc_item->physaddr >> PAGE_SHIFT;

		ret = ( remap_pfn_range( vma, ((u32)descriptor->mapping) + offset, linux_phys_frame_num, size, vma->vm_page_prot) ) ? _MALI_OSK_ERR_FAULT : _MALI_OSK_ERR_OK;

		if ( ret != _MALI_OSK_ERR_OK)
		{
			MALI_PRINT_ERROR(("%s %d could not remap_pfn_range()\n", __FUNCTION__, __LINE__));
			_allocation_list_item_release(alloc_item);
			return ret;
		}

		
		if (NULL == mappingInfo->list)
		{
			mappingInfo->list = alloc_item;
		}
		else
		{
			mappingInfo->tail->next = alloc_item;
		}

		mappingInfo->tail = alloc_item;
		alloc_item->next = NULL;
		alloc_item->offset = offset;

		
		*phys_addr = alloc_item->physaddr;

		return ret;
	}

	

	
	MALI_DEBUG_ASSERT( 0 == ((*phys_addr) & ~_MALI_OSK_CPU_PAGE_MASK) );

	return ( remap_pfn_range( vma, ((u32)descriptor->mapping) + offset, *phys_addr >> PAGE_SHIFT, size, vma->vm_page_prot) ) ? _MALI_OSK_ERR_FAULT : _MALI_OSK_ERR_OK;

}

void _mali_osk_mem_mapregion_unmap( mali_memory_allocation * descriptor, u32 offset, u32 size, _mali_osk_mem_mapregion_flags_t flags )
{
	MappingInfo *mappingInfo;

   if (NULL == descriptor) return;

	MALI_DEBUG_ASSERT( 0 != (descriptor->flags & MALI_MEMORY_ALLOCATION_FLAG_MAP_INTO_USERSPACE) );

	MALI_DEBUG_ASSERT( 0 == (size & ~_MALI_OSK_CPU_PAGE_MASK) );

	MALI_DEBUG_ASSERT( 0 == (offset & ~_MALI_OSK_CPU_PAGE_MASK) );

	if (NULL == descriptor->mapping) return;

	if (size > (descriptor->size - offset))
	{
		MALI_DEBUG_PRINT(1,("_mali_osk_mem_mapregion_unmap: virtual memory area not large enough to unmap size %x from area 0x%x at offset 0x%x\n",
							size, descriptor->mapping, offset));
		return;
	}
	mappingInfo = (MappingInfo *)descriptor->process_addr_mapping_info;

	MALI_DEBUG_ASSERT_POINTER( mappingInfo );

	if ( 0 != (flags & _MALI_OSK_MEM_MAPREGION_FLAG_OS_ALLOCATED_PHYSADDR) )
	{
		while (size)
		{
			
			AllocationList *alloc = mappingInfo->list;
			AllocationList **prev = &(mappingInfo->list);

			while (NULL != alloc && alloc->offset != offset)
			{
				prev = &(alloc->next);
				alloc = alloc->next;
			}
			if (alloc == NULL) {
				MALI_DEBUG_PRINT(1, ("Unmapping memory that isn't mapped\n"));
				size -= _MALI_OSK_CPU_PAGE_SIZE;
				offset += _MALI_OSK_CPU_PAGE_SIZE;
				continue;
			}

			*prev = alloc->next;
			_allocation_list_item_release(alloc);

			
			size -= _MALI_OSK_CPU_PAGE_SIZE;
			offset += _MALI_OSK_CPU_PAGE_SIZE;
		}
	}

	

	return;
}
