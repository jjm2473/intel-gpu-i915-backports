#ifndef __BACKPORT_DMA_BUF_H
#define __BACKPORT_DMA_BUF_H

#include_next <linux/dma-buf.h>

/**
 * struct dma_buf_map - Pointer to IO/system memory
 * @vaddr_iomem:	The buffer's address if in I/O memory
 * @vaddr:		The buffer's address if in system memory
 * @is_iomem:		True if the buffer is located in I/O memory, or false
 *			otherwise.
 */
struct dma_buf_map {
	union {
		void __iomem *vaddr_iomem;
		void *vaddr;
	};
	bool is_iomem;
};

/**
 * dma_buf_map_clear - Clears a iosys mapping structure
 * @map:	The dma_buf_map structure
 *
 * Clears all fields to zero, including struct dma_buf_map.is_iomem, so
 * mapping structures that were set to point to I/O memory are reset for
 * system memory. Pointers are cleared to NULL. This is the default.
 */
static inline void dma_buf_map_clear(struct dma_buf_map *map)
{
	memset(map, 0, sizeof(*map));
}

static inline int LINUX_I915_BACKPORT(dma_buf_vmap)(struct dma_buf *dmabuf, struct dma_buf_map *map) {
        void *dma_map;
        dma_buf_map_clear(map);
        dma_map = dma_buf_vmap(dmabuf);
        if (!dma_map) {
                return -ENOMEM;
        }
        map->vaddr = dma_map;
        return 0;
}

static inline void LINUX_I915_BACKPORT(dma_buf_vunmap)(struct dma_buf *dmabuf, struct dma_buf_map *map) {
        dma_buf_vunmap(dmabuf, map->vaddr);
}

#define dma_buf_vmap LINUX_I915_BACKPORT(dma_buf_vmap)
#define dma_buf_vunmap LINUX_I915_BACKPORT(dma_buf_vunmap)

#endif /* __BACKPORT_DMA_BUF_H */
