#ifndef _LINUX_FS_H
#define _LINUX_FS_H

/*
 * This file has definitions for some important file table
 * structures etc.
 */

#include <linux/config.h>
#include <linux/limits.h>
#include <linux/ioctl.h>

/*
 * It's silly to have NR_OPEN bigger than NR_FILE, but you can change
 * the file limit at runtime and only root can increase the per-process
 * nr_file rlimit, so it's safe to set up a ridiculously high absolute
 * upper limit on files-per-process.
 *
 * Some programs (notably those using select()) may have to be 
 * recompiled to take full advantage of the new limits..  
 */

/* Fixed constants first: */
#undef NR_OPEN
#define NR_OPEN (1024 * 1024) /* Absolute upper limit on fd num */
#define INR_OPEN 1024         /* Initial setting for nfile rlimits */

#define BLOCK_SIZE_BITS 10
#define BLOCK_SIZE (1 << BLOCK_SIZE_BITS)

/* And dynamically-tunable limits and defaults: */
struct files_stat_struct {
    int nr_files;      /* read only */
    int nr_free_files; /* read only */
    int max_files;     /* tunable */
};
extern struct files_stat_struct files_stat;

struct inodes_stat_t {
    int nr_inodes;
    int nr_unused;
    int dummy[5];
};
extern struct inodes_stat_t inodes_stat;

extern int leases_enable, lease_break_time;

#ifdef CONFIG_DNOTIFY
extern int dir_notify_enable;
#endif

#define NR_FILE 8192 /* this can well be larger on a larger system */

#define MAY_EXEC 1
#define MAY_WRITE 2
#define MAY_READ 4
#define MAY_APPEND 8

#define FMODE_READ 1
#define FMODE_WRITE 2

/* Internal kernel extensions */
#define FMODE_LSEEK 4
#define FMODE_PREAD 8
#define FMODE_PWRITE FMODE_PREAD /* These go hand in hand */

#define RW_MASK 1
#define RWA_MASK 2
#define READ 0
#define WRITE 1
#define READA 2   /* read-ahead  - don't block if no resources */
#define SPECIAL 4 /* For non-blockdevice requests in request queue */
#define READ_SYNC (READ | (1 << BIO_RW_SYNC))
#define WRITE_SYNC (WRITE | (1 << BIO_RW_SYNC))
#define WRITE_BARRIER ((1 << BIO_RW) | (1 << BIO_RW_BARRIER))

#define SEL_IN 1
#define SEL_OUT 2
#define SEL_EX 4

/* public flags for file_system_type */
/**
 * �������͵��ļ�ϵͳ����λ�����������ϡ�
 */
#define FS_REQUIRES_DEV 1
/**
 * ʹ�ö����ư�װ����?
 */
#define FS_BINARY_MOUNTDATA 2
/**
 * ��Ҫ���"."��".."������NFS
 */
#define FS_REVAL_DOT 16384 /* Check the paths ".", ".." for staleness */
/**
 * ����������ƶ�������NFS
 */
#define FS_ODD_RENAME                                                          \
    32768 /* Temporary stuff; will go away as soon
				  * as nfs_rename() will be cleaned up
				  */
/*
 * These are the fs-independent mount-flags: up to 32 flags are supported
 */
/**
 * ֻ��mount
 */
#define MS_RDONLY 1 /* Mount read-only */
/**
 * ��ֹsetuid��setgid��־��
 */
#define MS_NOSUID 2 /* Ignore suid and sgid bits */
/**
 * ��ֹ�����豸�ļ���
 */
#define MS_NODEV 4 /* Disallow access to device special files */
/**
 * �������ļ�ִ�С�
 */
#define MS_NOEXEC 8 /* Disallow program execution */
/**
 * �ļ���Ŀ¼�ϵ�д�����Ǽ�ʱ�ġ���mountʱָ����sync������
 */
#define MS_SYNCHRONOUS 16 /* Writes are synced at once */
/**
 * ���°�װ�ļ�ϵͳ��
 */
#define MS_REMOUNT 32 /* Alter flags of a mounted FS */
/**
 * ����ǿ�Ƽ�����
 */
#define MS_MANDLOCK 64 /* Allow mandatory locks on an FS */
/**
 * Ŀ¼�ϵ�д�����Ǽ�ʱ�ġ�
 */
#define MS_DIRSYNC 128 /* Directory modifications are synchronous */
/**
 * �������ļ�����ʱ�䡣
 */
#define MS_NOATIME 1024 /* Do not update access times. */
/**
 * ������Ŀ¼����ʱ�䡣
 */
#define MS_NODIRATIME 2048 /* Do not update directory access times */
/**
 * ����һ��"�󶨰�װ"������һ���ļ���Ŀ¼��ϵͳ������һ�����Ͽ��Ա��������μ�mount�����__bindѡ�
 */
#define MS_BIND 4096
/**
 * �Զ���һ���Ѱ�װ�ļ�ϵͳ�ƶ�������һ����װ�㡣�μ�mount�����__moveѡ�
 */
#define MS_MOVE 8192
/**
 * ΪĿ¼�����ݹ�ش���"�󶨰�װ"
 */
#define MS_REC 16384
/**
 * ����ʱ������ϸ���ں���Ϣ��
 */
#define MS_VERBOSE 32768 // 详细模式
//虚拟文件系统（VFS）不会应用用户文件创建掩码（umask）/* VFS does not apply the umask */
#define MS_POSIXACL (1 << 16)

#define MS_ACTIVE (1 << 30) //
#define MS_NOUSER (1 << 31)
/*
 * Superblock flags that can be altered by MS_REMOUNT
 */
#define MS_RMT_MASK                                                            \
    (MS_RDONLY | MS_SYNCHRONOUS | MS_MANDLOCK | MS_NOATIME | MS_NODIRATIME)

/*
 * Old magic mount flag and mask
 */
#define MS_MGC_VAL 0xC0ED0000
#define MS_MGC_MSK 0xffff0000

/* Inode flags - they have nothing to superblock flags now */

#define S_SYNC 1       /* Writes are synced at once */
#define S_NOATIME 2    /* Do not update access times */
#define S_APPEND 4     /* Append-only file */
#define S_IMMUTABLE 8  /* Immutable file */
#define S_DEAD 16      /* removed, but still open directory */
#define S_NOQUOTA 32   /* Inode is not counted to quota */
#define S_DIRSYNC 64   /* Directory modifications are synchronous */
#define S_NOCMTIME 128 /* Do not update file c/mtime */
#define S_SWAPFILE 256 /* Do not truncate: swapon got its bmaps */

/*
 * Note that nosuid etc flags are inode-specific: setting some file-system
 * flags just means all the inodes inherit those flags by default. It might be
 * possible to override it selectively if you really wanted to with some
 * ioctl() that is not currently implemented.
 *
 * Exception: MS_RDONLY is always applied to the entire file system.
 *
 * Unfortunately, it is possible to change a filesystems flags with it mounted
 * with files in use.  This means that all of the inodes will not have their
 * i_flags updated.  Hence, i_flags no longer inherit the superblock mount
 * flags, so these have to be checked separately. -- rmk@arm.uk.linux.org
 */
#define __IS_FLG(inode, flg) ((inode)->i_sb->s_flags & (flg))

#define IS_RDONLY(inode) ((inode)->i_sb->s_flags & MS_RDONLY)
#define IS_SYNC(inode)                                                         \
    (__IS_FLG(inode, MS_SYNCHRONOUS) || ((inode)->i_flags & S_SYNC))
#define IS_DIRSYNC(inode)                                                      \
    (__IS_FLG(inode, MS_SYNCHRONOUS | MS_DIRSYNC)                              \
     || ((inode)->i_flags & (S_SYNC | S_DIRSYNC)))
#define IS_MANDLOCK(inode) __IS_FLG(inode, MS_MANDLOCK)

#define IS_NOQUOTA(inode) ((inode)->i_flags & S_NOQUOTA)
#define IS_APPEND(inode) ((inode)->i_flags & S_APPEND)
#define IS_IMMUTABLE(inode) ((inode)->i_flags & S_IMMUTABLE)
#define IS_NOATIME(inode)                                                      \
    (__IS_FLG(inode, MS_NOATIME) || ((inode)->i_flags & S_NOATIME))
#define IS_NODIRATIME(inode) __IS_FLG(inode, MS_NODIRATIME)
#define IS_POSIXACL(inode) __IS_FLG(inode, MS_POSIXACL)

#define IS_DEADDIR(inode) ((inode)->i_flags & S_DEAD)
#define IS_NOCMTIME(inode) ((inode)->i_flags & S_NOCMTIME)
#define IS_SWAPFILE(inode) ((inode)->i_flags & S_SWAPFILE)

/* the read-only stuff doesn't really belong here, but any other place is
   probably as bad and I don't want to create yet another include file. */

#define BLKROSET _IO(0x12, 93)   /* set device read-only (0 = read-write) */
#define BLKROGET _IO(0x12, 94)   /* get read-only status (0 = read_write) */
#define BLKRRPART _IO(0x12, 95)  /* re-read partition table */
#define BLKGETSIZE _IO(0x12, 96) /* return device size /512 (long *arg) */
#define BLKFLSBUF _IO(0x12, 97)  /* flush buffer cache */
#define BLKRASET _IO(0x12, 98)   /* set read ahead for block device */
#define BLKRAGET _IO(0x12, 99)   /* get current read ahead setting */
#define BLKFRASET _IO(0x12, 100) /* set filesystem (mm/filemap.c) read-ahead */
#define BLKFRAGET _IO(0x12, 101) /* get filesystem (mm/filemap.c) read-ahead */
#define BLKSECTSET                                                             \
    _IO(0x12, 102) /* set max sectors per request (ll_rw_blk.c) */
#define BLKSECTGET                                                             \
    _IO(0x12, 103)               /* get max sectors per request (ll_rw_blk.c) */
#define BLKSSZGET _IO(0x12, 104) /* get block device sector size */
#if 0
#    define BLKPG _IO(0x12, 105) /* See blkpg.h */

/* Some people are morons.  Do not use sizeof! */

#    define BLKELVGET _IOR(0x12, 106, size_t) /* elevator get */
#    define BLKELVSET _IOW(0x12, 107, size_t) /* elevator set */
/* This was here just to show that the number is taken -
   probably all these _IO(0x12,*) ioctls should be moved to blkpg.h. */
#endif
/* A jump here: 108-111 have been used for various private purposes. */
#define BLKBSZGET _IOR(0x12, 112, size_t)
#define BLKBSZSET _IOW(0x12, 113, size_t)
#define BLKGETSIZE64                                                           \
    _IOR(0x12, 114, size_t) /* return device size in bytes (u64 *arg) */

#define BMAP_IOCTL 1          /* obsolete - kept for compatibility */
#define FIBMAP _IO(0x00, 1)   /* bmap access */
#define FIGETBSZ _IO(0x00, 2) /* get the block size used for bmap */

#ifdef __KERNEL__

#    include <linux/linkage.h>
#    include <linux/wait.h>
#    include <linux/types.h>
#    include <linux/kdev_t.h>
#    include <linux/dcache.h>
#    include <linux/stat.h>
#    include <linux/cache.h>
#    include <linux/kobject.h>
#    include <linux/list.h>
#    include <linux/radix-tree.h>
#    include <linux/prio_tree.h>
#    include <linux/audit.h>
#    include <linux/init.h>

#    include <asm/atomic.h>
#    include <asm/semaphore.h>
#    include <asm/byteorder.h>

struct iovec;
struct nameidata;
struct pipe_inode_info;
struct poll_table_struct;
struct kstatfs;
struct vm_area_struct;
struct vfsmount;

/* Used to be a macro which just called the function, now just a function */
extern void update_atime(struct inode *);

extern void __init inode_init(unsigned long);
extern void __init inode_init_early(void);
extern void __init mnt_init(unsigned long);
extern void __init files_init(unsigned long);

struct buffer_head;
typedef int(get_block_t)(struct inode *inode, sector_t iblock,
                         struct buffer_head *bh_result, int create);
typedef int(get_blocks_t)(struct inode *inode, sector_t iblock,
                          unsigned long max_blocks,
                          struct buffer_head *bh_result, int create);
typedef void(dio_iodone_t)(struct inode *inode, loff_t offset, ssize_t bytes,
                           void *private);

/*
 * Attribute flags.  These should be or-ed together to figure out what
 * has been changed!
 */
#    define ATTR_MODE 1
#    define ATTR_UID 2
#    define ATTR_GID 4
#    define ATTR_SIZE 8
#    define ATTR_ATIME 16
#    define ATTR_MTIME 32
#    define ATTR_CTIME 64
#    define ATTR_ATIME_SET 128
#    define ATTR_MTIME_SET 256
#    define ATTR_FORCE 512 /* Not a change, but a change it */
#    define ATTR_ATTR_FLAG 1024
#    define ATTR_KILL_SUID 2048
#    define ATTR_KILL_SGID 4096

/*
 * This is the Inode Attributes structure, used for notify_change().  It
 * uses the above definitions as flags, to know which values have changed.
 * Also, in this manner, a Filesystem can look at only the values it cares
 * about.  Basically, these are the attributes that the VFS layer can
 * request to change from the FS layer.
 *
 * Derek Atkins <warlord@MIT.EDU> 94-10-20
 */
/**
 * NFS�ͻ����޸ķ��������ļ�����ʱ�õ��Ľṹ��
 */
struct iattr {
    unsigned int ia_valid;
    /**
	 * �ļ�����λ 
	 */
    umode_t ia_mode;
    /** 
	 * �ļ�ӵ���ߵ�id 
	 */
    uid_t ia_uid;
    /** 
	 * �ļ��������� 
	 */
    gid_t ia_gid;
    /**
	 * �ļ���С 
	 */
    loff_t ia_size;
    /**
	 * �ļ���������ʱ�� 
	 */
    struct timespec ia_atime;
    /**
	 * �ļ�������޸�ʱ�� 
	 */
    struct timespec ia_mtime;
    struct timespec ia_ctime;
    unsigned int ia_attr_flags;
};

/*
 * This is the inode attributes flag definitions
 */
#    define ATTR_FLAG_SYNCRONOUS 1  /* Syncronous write */
#    define ATTR_FLAG_NOATIME 2     /* Don't update atime */
#    define ATTR_FLAG_APPEND 4      /* Append-only file */
#    define ATTR_FLAG_IMMUTABLE 8   /* Immutable file */
#    define ATTR_FLAG_NODIRATIME 16 /* Don't update atime for directory */

/*
 * Includes for diskquotas.
 */
#    include <linux/quota.h>

/*
 * oh the beauties of C type declarations.
 */
struct page;
struct address_space;
struct writeback_control;
struct kiocb;

/**
 * ��ҳ���д����ĸ��ַ���
 */
struct address_space_operations {
    /**
	 * д����(��ҳд�������ߵĴ���ӳ��)
	 */
    int (*writepage)(struct page *page, struct writeback_control *wbc);
    /**
	 * ������(�������ߵĴ���ӳ�����ҳ)
	 */
    int (*readpage)(struct file *, struct page *);
    /**
	 * �����������ҳ���еĲ�����׼���ã������̿�ʼI/O���ݵĴ���
	 */
    int (*sync_page)(struct page *);

    /* Write back some dirty pages from this mapping. */
    /**
	 * ��ָ����������������ҳд�ش���
	 */
    int (*writepages)(struct address_space *, struct writeback_control *);

    /* Set a page dirty */
    /**
	 * �������ߵ�ҳ����Ϊ��ҳ
	 */
    int (*set_page_dirty)(struct page *page);

    /**
	 * �Ӵ����ж�������ҳ������
	 */
    int (*readpages)(struct file *filp, struct address_space *mapping,
                     struct list_head *pages, unsigned nr_pages);

    /*
	 * ext3 requires that a successful prepare_write() call be followed
	 * by a commit_write() call - they must be balanced
	 */
    /**
	 * Ϊд������׼�����ɴ����ļ�ϵͳʹ�ã�
	 */
    int (*prepare_write)(struct file *, struct page *, unsigned, unsigned);
    /**
	 * ���д�������ɴ����ļ�ϵͳʹ�ã�
	 */
    int (*commit_write)(struct file *, struct page *, unsigned, unsigned);
    /* Unfortunately this kludge is needed for FIBMAP. Don't use it */
    /**
	 * ���ļ��������л�ȡ�߼����
	 */
    sector_t (*bmap)(struct address_space *, sector_t);
    /**
	 * ʹ�����ߵ�ҳ��Ч���ض��ļ�ʱ�ã�
	 */
    int (*invalidatepage)(struct page *, unsigned long);
    /**
	 * ����־�ļ�ϵͳʹ�ã���׼���ͷ�ҳ
	 */
    int (*releasepage)(struct page *, int);
    /**
	 * ������ҳ��ֱ��I/O����(�ƹ�ҳ���ٻ���)
	 */
    ssize_t (*direct_IO)(int, struct kiocb *, const struct iovec *iov,
                         loff_t offset, unsigned long nr_segs);
};

struct backing_dev_info;
/**
 * ҳ���ٻ���ĺ������ݽṹ
 * ����һ��Ƕ����ҳ�����ߵ������������е����ݽṹ
 * (ҳ���������ܻ�����ȱҳ�쳣����Щ��������ҳӵ���ڲ����κ���������еĹ���address_space������)
 * ���ٻ������е�����ҳ��������ͬһ�������ߣ��Ӷ����ܱ����ӵ�ͬһ��address_space�����С�
 * �ö����������ߵ�ҳ�Ͷ���Щҳ�Ĳ���֮�佨�������ӹ�ϵ��
 */
struct address_space {
    /**
	 * ������ڣ���ָ��ӵ�иö������������ָ�롣
	 */
    struct inode *host; /* owner: inode, block_device */
    /**
	 * ӵ���ߵ�ҳ�Ļ���
	 */
    struct radix_tree_root page_tree; /* radix tree of all pages */
    /**
	 * ����������������
	 */
    spinlock_t tree_lock; /* and spinlock protecting it */
    /**
	 * ��ַ�ռ��й����ڴ�ӳ��ĸ�����
	 */
    unsigned int i_mmap_writable; /* count VM_SHARED mappings */
    /**
	 * radix�����������ĸ�������ӳ��ҳ(�繲�������ļ�������C��)�ķ���ӳ�䡣
	 */
    struct prio_tree_root i_mmap; /* tree of private and shared mappings */
    /**
	 * ��ַ�ռ��з������ڴ���������
	 */
    struct list_head i_mmap_nonlinear; /*list VM_NONLINEAR mappings */
    /**
	 * ����radix��������������������
	 */
    spinlock_t i_mmap_lock; /* protect tree, count, list */
    /**
	 * �ض��ļ�ʱʹ�õ�˳���������
	 */
    unsigned int truncate_count; /* Cover race condition with truncate */
    /**
	 * �����ߵ�ҳ������
	 */
    unsigned long nrpages; /* number of total pages */
    /**
	 * ���һ�λ�д���������õ�ҳ������
	 */
    pgoff_t writeback_index; /* writeback starts here */
    /**
	 * ��������ҳ���в����ķ���
	 */
    struct address_space_operations *a_ops; /* methods */
    /**
	 * ����λ���ڴ�������ı�־
	 */
    unsigned long flags; /* error bits/gfp mask */
    /**
	 * ָ��ӵ�����������ݵĿ��豸��backing_dev_infoָ��
	 */
    struct backing_dev_info *backing_dev_info; /* device readahead, etc */
    /**
	 * ͨ���ǹ���private_list����ʱʹ�õ�������
	 * ���¼����ֶο����ɸ��ļ�ϵͳ����ʹ�á�
	 */
    spinlock_t private_lock; /* for use by the address_space */
    /**
	 * ͨ���������������صļ�ӿ���໺����������
	 */
    struct list_head private_list; /* ditto */
    /**
	 * ͨ����ָ���ӿ����ڿ��豸��address_space�����ָ��
	 */
    struct address_space *assoc_mapping; /* ditto */
} __attribute__((aligned(sizeof(long))));
/*
	 * On most architectures that alignment is already the case; but
	 * must be enforced here for CRIS, to let the least signficant bit
	 * of struct page's "mapping" pointer be used for PAGE_MAPPING_ANON.
	 */
/**
 * һ�����豸����������Դ����������豸.
 * ���磺һ��IDE����������Դ�������IDE���̡����е�ÿ������һ�������Ŀ��豸��
 * ���ң�ÿ�����̶����Ա�������ÿ�������ֿ��Ա�������һ���߼��豸��
 * ÿ�����豸��������block_device����ġ�
 */
struct block_device {
    /**
	 * ���豸�����豸�źʹ��豸��
	 */
    dev_t bd_dev; /* not a kdev_t - it's a search key */
    /**
	 * ָ��bdev�ļ�ϵͳ�п��豸��Ӧ���ļ���������ָ�롣
	 */
    struct inode *bd_inode; /* will die */
    /**
	 * ��������ͳ���豸�Ѿ������˶��ٴ�
	 */
    int bd_openers;
    /**
	 * �������豸�򿪺͹رյ��ź�����
	 */
    struct semaphore bd_sem; /* open/close mutex */
    /**
	 * ��ֹ�ڿ��豸�Ͻ����°�װ(mount)���ź�����
	 */
    struct semaphore bd_mount_sem; /* mount mutex */
    /**
	 * �Ѵ򿪵Ŀ��豸�ļ�����������������ײ���
	 */
    struct list_head bd_inodes;
    /**
	 * ���豸�������ĵ�ǰ������
	 */
    void *bd_holder;
    /**
	 * ��������ͳ�ƶ�bd_holder�ֶζ�����õĴ�����
	 */
    int bd_holders;
    /**
	 * ����豸��һ����������ָ���������̵Ŀ��豸��������
	 * ����ָ��ÿ��豸������
	 */
    struct block_device *bd_contains;
    /**
	 * ���С 
	 */
    unsigned bd_block_size;
    /**
	 * ָ�������������ָ�루������豸���Ƿ�������ΪNULL��
	 */
    struct hd_struct *bd_part;
    /* number of times partitions within this device have been opened. */
    /**
	 * ��������ͳ�ư����ڿ��豸�еķ����Ѿ������˶��ٴ�
	 */
    unsigned bd_part_count;
    /**
	 * ����Ҫ�����豸�ķ�����ʱ���õı�־
	 */
    int bd_invalidated;
    /**
	 * ָ����豸�л������̵�gendisk�ṹ��ָ��
	 */
    struct gendisk *bd_disk;
    /**
	 * ���ڿ��豸������������ָ��
	 */
    struct list_head bd_list;
    /**
	 * ָ����豸��ר����������ͨ��ΪNULL��
	 */
    struct backing_dev_info *bd_inode_backing_dev_info;
    /*
	 * Private data.  You must have bd_claim'ed the block_device
	 * to use this.  NOTE:  bd_claim allows an owner to claim
	 * the same device multiple times, the owner must take special
	 * care to not mess up bd_private for that case.
	 */
    /**
	 * ���豸�����ߵ�˽������ָ��
	 */
    unsigned long bd_private;
};

/*
 * Radix-tree tags, for tagging dirty and writeback pages within the pagecache
 * radix trees
 */
#    define PAGECACHE_TAG_DIRTY 0
#    define PAGECACHE_TAG_WRITEBACK 1

int mapping_tagged(struct address_space *mapping, int tag);

/*
 * Might pages of this file be mapped into userspace?
 */
static inline int mapping_mapped(struct address_space *mapping)
{
    return !prio_tree_empty(&mapping->i_mmap)
           || !list_empty(&mapping->i_mmap_nonlinear);
}

/*
 * Might pages of this file have been modified in userspace?
 * Note that i_mmap_writable counts all VM_SHARED vmas: do_mmap_pgoff
 * marks vma as VM_SHARED if it is shared, and the file was opened for
 * writing i.e. vma may be mprotected writable even if now readonly.
 */
static inline int mapping_writably_mapped(struct address_space *mapping)
{
    return mapping->i_mmap_writable != 0;
}

/*
 * Use sequence counter to get consistent i_size on 32-bit processors.
 */
#    if BITS_PER_LONG == 32 && defined(CONFIG_SMP)
#        include <linux/seqlock.h>
#        define __NEED_I_SIZE_ORDERED
#        define i_size_ordered_init(inode)                                     \
            seqcount_init(&inode->i_size_seqcount)
#    else
#        define i_size_ordered_init(inode)                                     \
            do {                                                               \
            } while(0)
#    endif

/**
 * �ں��øýṹ���ڲ���ʾһ���ļ�������file��ͬ��file��ʾ�ĵ��ļ���������
 * �Ե����ļ������ܻ����������ʾ�򿪵��ļ���������filep�ṹ���������Ƕ�ָ�򵥸�inode�ṹ��
 */
struct inode {
    /**
	 * ͨ�����ֶν���������ϣ����
	 */
    struct hlist_node i_hash;
    /**
	 * ͨ�����ֶν��������벻ͬ״̬�������С�
	 */
    struct list_head i_list;
    /**
	 * ͨ�����ֶν������뵽�������inode�����С�
	 */
    struct list_head i_sb_list;
    /**
	 * ���������ڵ��Ŀ¼���������ͷ��
	 */
    struct list_head i_dentry;
    /**
	 * �����ڵ��š�
	 */
    unsigned long i_ino;
    /**
	 * ���ü�������
	 */
    atomic_t i_count;
    /**
	 * �ļ����������Ȩ�ޡ�
	 */
    umode_t i_mode;
    /**
	 * Ӳ������Ŀ��
	 */
    unsigned int i_nlink;
    /**
	 * ������ID
	 */
    uid_t i_uid;
    /**
	 * ���������ʶ����
	 */
    gid_t i_gid;
    /**
	 * �Ա�ʾ�豸�ļ���inode�ṹ�����ֶΰ������������豸��š�
	 */
    dev_t i_rdev;
    /**
	 * �ļ����ֽ�����
	 */
    loff_t i_size;
    /**
	 * �ϴη����ļ���ʱ�䡣
	 */
    struct timespec i_atime;
    /**
	 * �ϴ����ļ���ʱ�䡣
	 */
    struct timespec i_mtime;
    /**
	 * �ϴ��޸������ڵ��ʱ�䡣
	 */
    struct timespec i_ctime;
    /**
	 * ���λ����
	 */
    unsigned int i_blkbits;
    /**
	 * ����ֽ�����
	 */
    unsigned long i_blksize;
    /**
	 * �汾�ţ�ÿ��ʹ�ú������
	 */
    unsigned long i_version;
    /**
	 * �ļ��Ŀ�����
	 */
    unsigned long i_blocks;
    /**
	 * �ļ����һ������ֽ�����
	 */
    unsigned short i_bytes;
    /**
	 * ��0��ʾ�ļ���һ���׽��֡�
	 */
    unsigned char i_sock;
    /**
	 * ���������ڵ�ĳЩ�ֶε���������
	 */
    spinlock_t i_lock; /* i_blocks, i_bytes, maybe i_size */
    /**
	 * ���������ڵ���ź�����
	 */
    struct semaphore i_sem;
    /**
	 * ��ֱ��IO�ļ������б�����־��������Ķ�д�ź�����
	 */
    struct rw_semaphore i_alloc_sem;
    /**
	 * �����ڵ�Ĳ�����
	 */
    struct inode_operations *i_op;
    /**
	 * ȱʡ�ļ�������
	 */
    struct file_operations *i_fop; /* former ->i_op->default_file_ops */
    /**
	 * inode���ڵĳ����顣
	 */
    struct super_block *i_sb;
    /**
	 * �ļ�������,ͨ�����ֶν��ļ��ϵ����������ӳ�һ����������
	 */
    struct file_lock *i_flock;
    /**
	 * ָ��address_space�����ָ�롣
	 */
    struct address_space *i_mapping;
    /**
	 * �ļ���address_space����
	 */
    struct address_space i_data;
#    ifdef CONFIG_QUOTA
    /**
	 * �����ڵ�Ĵ����޶
	 */
    struct dquot *i_dquot[MAXQUOTAS];
#    endif
    /* These three should probably be a union */
    /**
	 * ���ھ�����ַ�����豸�������ڵ�����ָ�롣
	 */
    struct list_head i_devices;
    /**
	 * ����ں���һ���ܵ����0
	 */
    struct pipe_inode_info *i_pipe;
    /**
	 * ָ����豸���������ָ�롣
	 */
    struct block_device *i_bdev;
    /**
	 * ��ʾ�ַ��豸���ڲ����ݽṹ����inodeָ��һ���ַ��豸�ļ�ʱ�����ֶΰ�����ָ��struct cdev�ṹ��ָ�롣
	 */
    struct cdev *i_cdev;
    /**
	 * ���豸��������
	 */
    int i_cindex;

    /**
	 * �����ڵ�汾�š���ĳЩ�ļ�ϵͳʹ�á�
	 */
    __u32 i_generation;

#    ifdef CONFIG_DNOTIFY
    /**
	 * Ŀ¼֪ͨ�¼����롣
	 */
    unsigned long i_dnotify_mask; /* Directory notify events */
    /**
	 * ����Ŀ¼֪ͨ��
	 */
    struct dnotify_struct *i_dnotify; /* for directory notifications */
#    endif

    /**
	 * �����ڵ�״̬��־��
	 */
    unsigned long i_state;
    /**
	 * �����ڵ�Ū���ʱ�䣬��jiffiesΪ��λ��
	 */
    unsigned long dirtied_when; /* jiffies of first dirtying */

    /**
	 * �ļ�ϵͳ�İ�װ��־��
	 */
    unsigned int i_flags;

    /**
	 * ����д���̵����ü�����
	 */
    atomic_t i_writecount;
    /**
	 * �����ڵ㰲ȫ�ṹ��
	 */
    void *i_security;
    /**
	 * �ļ�ϵͳ˽������ָ�롣
	 */
    union {
        void *generic_ip;
    } u;
#    ifdef __NEED_I_SIZE_ORDERED
    /**
	 * SMPϵͳΪi_size�ֶλ�ȡһ����ʱʹ�õ�˳���������
	 */
    seqcount_t i_size_seqcount;
#    endif
};

/*
 * NOTE: in a 32bit arch with a preemptable kernel and
 * an UP compile the i_size_read/write must be atomic
 * with respect to the local cpu (unlike with preempt disabled),
 * but they don't need to be atomic with respect to other cpus like in
 * true SMP (so they need either to either locally disable irq around
 * the read or for example on x86 they can be still implemented as a
 * cmpxchg8b without the need of the lock prefix). For SMP compiles
 * and 64bit archs it makes no difference if preempt is enabled or not.
 */
static inline loff_t i_size_read(struct inode *inode)
{
#    if BITS_PER_LONG == 32 && defined(CONFIG_SMP)
    loff_t i_size;
    unsigned int seq;

    do {
        seq    = read_seqcount_begin(&inode->i_size_seqcount);
        i_size = inode->i_size;
    } while(read_seqcount_retry(&inode->i_size_seqcount, seq));
    return i_size;
#    elif BITS_PER_LONG == 32 && defined(CONFIG_PREEMPT)
    loff_t i_size;

    preempt_disable();
    i_size = inode->i_size;
    preempt_enable();
    return i_size;
#    else
    return inode->i_size;
#    endif
}

static inline void i_size_write(struct inode *inode, loff_t i_size)
{
#    if BITS_PER_LONG == 32 && defined(CONFIG_SMP)
    write_seqcount_begin(&inode->i_size_seqcount);
    inode->i_size = i_size;
    write_seqcount_end(&inode->i_size_seqcount);
#    elif BITS_PER_LONG == 32 && defined(CONFIG_PREEMPT)
    preempt_disable();
    inode->i_size = i_size;
    preempt_enable();
#    else
    inode->i_size = i_size;
#    endif
}

/**
 * ��inode�л�����豸��
 */
static inline unsigned iminor(struct inode *inode)
{
    return MINOR(inode->i_rdev);
}

/**
 * ��inode�л�ô��豸��
 */
static inline unsigned imajor(struct inode *inode)
{
    return MAJOR(inode->i_rdev);
}

extern struct block_device *I_BDEV(struct inode *inode);

struct fown_struct {
    rwlock_t lock;   /* protects pid, uid, euid fields */
    int pid;         /* pid or -pgrp where SIGIO should be sent */
    uid_t uid, euid; /* uid/euid of process setting the owner */
    void *security;
    int signum; /* posix.1b rt signal to be delivered on IO */
};

/*
 * Track a single file's readahead state
 */
/**
 * Ԥ���㷨ʹ�õ���Ҫ���ݽṹ.ÿ���ļ�����������f_ra�ֶ��д�Ÿ���������
 */
struct file_ra_state {
    /**
	 * ��ǰ���ڵ�һҳ��������
	 */
    unsigned long start; /* Current window */
    /**
	 * ��ǰ���ڵ�ҳ������ֹԤ��ʱΪ��1��0��ʾ��ǰ��Ϊ��
	 */
    unsigned long size;
    /**
	 * ����Ԥ���ı�־��
	 */
    unsigned long flags; /* ra flags RA_FLAG_xxx*/
    /**
	 * �������ٻ���������(���������ҳͬʱ����ҳ���ٻ�����)
	 */
    unsigned long cache_hit; /* cache hit count*/
    /**
	 * ������������һҳ��������
	 */
    unsigned long prev_page; /* Cache last read() position */
    /**
	 * Ԥ�����ڵĵ�һҳ������
	 */
    unsigned long ahead_start; /* Ahead window */
    /**
	 * Ԥ������ҳ��(0��ʾԤ�����ڿ�)
	 */
    unsigned long ahead_size;
    /**
	 * Ԥ�����ڵ����ҳ��(0��ʾԤ�������ý�ֹ)
	 * ���ֶεĳ�ʼֵ(ȱʡֵ)����ڸ��ļ����ڿ��豸��backing_dev_info��������
	 * Ӧ�ÿ���ͨ������posix_fadviseϵͳ�����޸�һ�����ļ���ra_pages�ֶΡ�
	 */
    unsigned long ra_pages; /* Maximum readahead window */
    /**
	 * Ԥ�����м�����(�����ڴ�ӳ���ļ�)
	 */
    unsigned long mmap_hit; /* Cache hit stat for mmap accesses */
    /**
	 * Ԥ��ʧ�ܼ�����(�����ڴ�ӳ���ļ�)
	 */
    unsigned long mmap_miss; /* Cache miss stat for mmap accesses */
};
/**
 * ����Ѿ���Ԥ����ҳ����ҳ���ٻ�����(�������ں�Ϊ���ͷ��ڴ�����Ի�����)����ñ�־����λ��
 * ��ʱ����һ��Ҫ������Ԥ�����ڴ�С������С��
 */
#    define RA_FLAG_MISS 0x01 /* a cache miss occured against this file */
/**
 * ���ں�ȷ��������������256��ҳ����ҳ���ٻ�����ʱ(�������ٻ��������������ra->cache_hit�ֶ�)����ñ�־λ��λ��
 * 
 */
#    define RA_FLAG_INCACHE 0x02 /* file is already in cache */

struct file {

    struct list_head f_list; // 打开文件列表中连接的多个struct file

    struct dentry *f_dentry; // 指向文件关联的目录项 
	
    struct vfsmount *f_vfsmnt; // 指向这个文件的挂载点,这个文件在哪个文件系统上

    struct file_operations *f_op; // 操作函数

    atomic_t f_count; // 引用计数

    unsigned int f_flags;

    mode_t f_mode;

    int f_error;

    loff_t f_pos;
    struct fown_struct f_owner;

    unsigned int f_uid, f_gid;

    struct file_ra_state f_ra;

    size_t f_maxcount;

    unsigned long f_version;

    void *f_security;

    /* needed for tty driver, and maybe others */
    /**
	 * openϵͳ�����ڵ������������open����ǰ�����ָ����ΪNULL������������Խ�����ֶ������κ�Ŀ�Ļ��ߺ�������ֶΡ�
	 * �����������������ֶ�ָ���ѷ�������ݣ�����һ��Ҫ���ں�����file�ṹǰ��release�������ͷ��ڴ档
	 * ���ǿ�ϵͳ����ʱ����״̬�ķǳ����õ���Դ��
	 */
    void *private_data;

#    ifdef CONFIG_EPOLL
    /* Used by fs/eventpoll.c to link all the hooks to this file */
    
    struct list_head f_ep_links;
    
    spinlock_t f_ep_lock;
#    endif /* #ifdef CONFIG_EPOLL */
   
    struct address_space *f_mapping;
};
extern spinlock_t files_lock;
#    define file_list_lock() spin_lock(&files_lock);
#    define file_list_unlock() spin_unlock(&files_lock);

#    define get_file(x) atomic_inc(&(x)->f_count)
#    define file_count(x) atomic_read(&(x)->f_count)

#    define MAX_NON_LFS ((1UL << 31) - 1)

/* Page cache limit. The filesystems should put that into their s_maxbytes 
   limits, otherwise bad things can happen in VM. */
#    if BITS_PER_LONG == 32
#        define MAX_LFS_FILESIZE                                               \
            (((u64)PAGE_CACHE_SIZE << (BITS_PER_LONG - 1)) - 1)
#    elif BITS_PER_LONG == 64
#        define MAX_LFS_FILESIZE 0x7fffffffffffffffUL
#    endif

#    define FL_POSIX 1
#    define FL_FLOCK 2
#    define FL_ACCESS 8  /* not trying to lock, just looking */
#    define FL_LOCKD 16  /* lock held by rpc.lockd */
#    define FL_LEASE 32  /* lease held on this file */
#    define FL_SLEEP 128 /* A blocking lock */

/*
 * The POSIX file lock owner is determined by
 * the "struct files_struct" in the thread group
 * (or NULL for no owner - BSD locks).
 *
 * Lockd stuffs a "host" pointer into this.
 */
typedef struct files_struct *fl_owner_t;

struct file_lock_operations {
    void (*fl_insert)(struct file_lock *); /* lock insertion callback */
    void (*fl_remove)(struct file_lock *); /* lock removal callback */
    void (*fl_copy_lock)(struct file_lock *, struct file_lock *);
    void (*fl_release_private)(struct file_lock *);
};

struct lock_manager_operations {
    int (*fl_compare_owner)(struct file_lock *, struct file_lock *);
    void (*fl_notify)(struct file_lock *); /* unblock callback */
    void (*fl_copy_lock)(struct file_lock *, struct file_lock *);
    void (*fl_release_private)(struct file_lock *);
    void (*fl_break)(struct file_lock *);
};

/* that will die - we need it for nfs_lock_info */
#    include <linux/nfs_fs_i.h>

/**
 * �ļ���
 */
struct file_lock {
    /**
	 * �ļ��е���һ������
	 */
    struct file_lock *fl_next; /* singly linked list for this inode  */
    /**
	 * �����ӵ����������������
	 */
    struct list_head fl_link; /* doubly linked list of all locks */
    /**
	 * �ȴ���������
	 */
    struct list_head fl_block; /* circular list of blocked processes */
    /**
	 * �ļ������ߵ�files_struct
	 */
    fl_owner_t fl_owner;
    /**
	 * ӵ���ߵ�PID
	 */
    unsigned int fl_pid;
    /**
	 * �������̵ĵȴ����У����еȴ������Ľ����ڴ˶����С�
	 */
    wait_queue_head_t fl_wait;
    /**
	 * ָ���ļ������ָ�롣
	 */
    struct file *fl_file;
    /**
	 * ����־
	 */
    unsigned char fl_flags;
    /**
	 * ������
	 */
    unsigned char fl_type;
    /**
	 * �����������ʼλ��
	 */
    loff_t fl_start;
    /**
	 * ��������Ľ���λ�á�
	 */
    loff_t fl_end;

    /**
	 * ����������ж�֪ͨ��
	 */
    struct fasync_struct *fl_fasync; /* for lease break notifications */
    /**
	 * ������ǰ��ʣ��ʱ��
	 */
    unsigned long fl_break_time; /* for nonblocking lease breaks */

    /**
	 * �ļ�������ָ��
	 */
    struct file_lock_operations *fl_ops; /* Callbacks for filesystems */
    /**
	 * ����������ָ�롣
	 */
    struct lock_manager_operations *fl_lmops; /* Callbacks for lockmanagers */
    /**
	 * �����ļ�ϵͳ����Ϣ��NFSʹ�á�
	 */
    union {
        struct nfs_lock_info nfs_fl;
    } fl_u;
};

/* The following constant reflects the upper bound of the file/locking space */
#    ifndef OFFSET_MAX
#        define INT_LIMIT(x) (~((x)1 << (sizeof(x) * 8 - 1)))
#        define OFFSET_MAX INT_LIMIT(loff_t)
#        define OFFT_OFFSET_MAX INT_LIMIT(off_t)
#    endif

/**
 * ���л��������
 */
extern struct list_head file_lock_list;

#    include <linux/fcntl.h>

extern int fcntl_getlk(struct file *, struct flock __user *);
extern int fcntl_setlk(struct file *, unsigned int, struct flock __user *);

#    if BITS_PER_LONG == 32
extern int fcntl_getlk64(struct file *, struct flock64 __user *);
extern int fcntl_setlk64(struct file *, unsigned int, struct flock64 __user *);
#    endif

extern void send_sigio(struct fown_struct *fown, int fd, int band);
extern int fcntl_setlease(unsigned int fd, struct file *filp, long arg);
extern int fcntl_getlease(struct file *filp);

/* fs/locks.c */
extern void locks_init_lock(struct file_lock *);
extern void locks_copy_lock(struct file_lock *, struct file_lock *);
extern void locks_remove_posix(struct file *, fl_owner_t);
extern void locks_remove_flock(struct file *);
extern struct file_lock *posix_test_lock(struct file *, struct file_lock *);
extern int posix_lock_file(struct file *, struct file_lock *);
extern int posix_lock_file_wait(struct file *, struct file_lock *);
extern void posix_block_lock(struct file_lock *, struct file_lock *);
extern void posix_unblock_lock(struct file *, struct file_lock *);
extern int posix_locks_deadlock(struct file_lock *, struct file_lock *);
extern int flock_lock_file_wait(struct file *filp, struct file_lock *fl);
extern int __break_lease(struct inode *inode, unsigned int flags);
extern void lease_get_mtime(struct inode *, struct timespec *time);
extern int setlease(struct file *, long, struct file_lock **);
extern void remove_lease(struct file_lock *);
extern int lock_may_read(struct inode *, loff_t start, unsigned long count);
extern int lock_may_write(struct inode *, loff_t start, unsigned long count);
extern void steal_locks(fl_owner_t from);

struct fasync_struct {
    int magic;
    int fa_fd;
    struct fasync_struct *fa_next; /* singly linked list */
    struct file *fa_file;
};

#    define FASYNC_MAGIC 0x4601

/* SMP safe fasync helpers: */
extern int fasync_helper(int, struct file *, int, struct fasync_struct **);
/* can be called from interrupts */
extern void kill_fasync(struct fasync_struct **, int, int);
/* only for net: no internal synchronization */
extern void __kill_fasync(struct fasync_struct *, int, int);

extern int f_setown(struct file *filp, unsigned long arg, int force);
extern void f_delown(struct file *filp);
extern int send_sigurg(struct fown_struct *fown);

/*
 *	Umount options
 */

#    define MNT_FORCE 0x00000001  /* Attempt to forcibily umount */
#    define MNT_DETACH 0x00000002 /* Just detach from the tree */
#    define MNT_EXPIRE 0x00000004 /* Mark for expiry */

// 超级块链表
extern struct list_head super_blocks;

extern spinlock_t sb_lock;

#    define sb_entry(list) list_entry((list), struct super_block, s_list)
#    define S_BIAS (1 << 30)
/**
 * 超级块结构体
 */
struct super_block {
    /************ 管理超级块的域 ****************/
    struct list_head s_list;

    unsigned char s_dirt; /* 脏位 */

    struct dentry *s_root; // 指向根目录的dentry 目录项

    int s_count; /* 对超级块的使用次数*/
    /*对超级块读写进行同步 */
    struct rw_semaphore s_umount;

    // 链表
    struct list_head s_files; // 文件链表

    /* 已经修改的inodes形成链表 */ /* dirty inodes */
    struct list_head s_dirty;
    /* 所有的inode*/
    struct list_head s_inodes; /* all inodes */

    /* **************描述具体文件系统的整体信息的域****************/
    /* 包含该具体文件系统的块设备标识符。例如，对于 /dev/hda1，其设备标识符为 0x301*/
    dev_t s_dev;
    /*该具体文件系统中数据块的大小， 以字节为单位 */
    unsigned long s_blocksize;
    /* 数据块大小（old）*/
    unsigned long s_old_blocksize;
    /*块大小的值占用的位数，例如，如果块大小为1024字节，则该值为10*/
    unsigned char s_blocksize_bits;
    /* 文件的最大长度*/ /* Max file size */
    unsigned long long s_maxbytes;
    /* 安装标志*/
    unsigned long s_flags;
    /* 魔数，即该具体文件系统区别于其它 文系统的一个标志*/
    unsigned long s_magic;

    /***********  具体文件系统的域*****************/

    /*指向文件系统的 file_system_type 数据结构的指针 */
    struct file_system_type *s_type;
    /* 超级块操作函数 */
    struct super_operations *s_op;
    /* 指向某个特定的具体文件系统   用于限额操作的函数集合 */
    struct dquot_operations *dq_op;

    struct quotactl_ops *s_qcop;

    struct export_operations *s_export_op;

    struct semaphore s_lock;

    void *s_fs_info; /* Filesystem private info */

    int s_syncing; // 同步

    int s_need_sync_fs;

    atomic_t s_active;

    void *s_security;

    struct xattr_handler **s_xattr;

    struct list_head s_io; /* parked for writeback */

    struct hlist_head s_anon; /* anonymous dentries for (nfs) exporting */

    struct block_device *s_bdev;

    struct list_head s_instances;

    struct quota_info s_dquot; /* Diskquota specific options */

    int s_frozen;

    wait_queue_head_t s_wait_unfrozen;

    char s_id[32]; /* Informational name */

    /*
	 * The next field is for VFS *only*. No filesystems have any business
	 * even looking at it. You had been warned.
	 */

    struct semaphore s_vfs_rename_sem; /* Kludge */

    /* Granuality of c/m/atime in ns.
	   Cannot be worse than a second */

    u32 s_time_gran;
};

extern struct timespec current_fs_time(struct super_block *sb);

/*
 * Snapshotting support.
 */
enum {
    SB_UNFROZEN     = 0,
    SB_FREEZE_WRITE = 1,
    SB_FREEZE_TRANS = 2,
};

#    define vfs_check_frozen(sb, level)                                        \
        wait_event((sb)->s_wait_unfrozen, ((sb)->s_frozen < (level)))

/*
 * Superblock locking.
 */
static inline void lock_super(struct super_block *sb)
{
    down(&sb->s_lock);
}

static inline void unlock_super(struct super_block *sb)
{
    up(&sb->s_lock);
}

/*
 * VFS helper functions..
 */
extern int vfs_create(struct inode *, struct dentry *, int, struct nameidata *);
extern int vfs_mkdir(struct inode *, struct dentry *, int);
extern int vfs_mknod(struct inode *, struct dentry *, int, dev_t);
extern int vfs_symlink(struct inode *, struct dentry *, const char *, int);
extern int vfs_link(struct dentry *, struct inode *, struct dentry *);
extern int vfs_rmdir(struct inode *, struct dentry *);
extern int vfs_unlink(struct inode *, struct dentry *);
extern int vfs_rename(struct inode *, struct dentry *, struct inode *,
                      struct dentry *);

/*
 * VFS dentry helper functions.
 */
extern void dentry_unhash(struct dentry *dentry);

/*
 * File types
 *
 * NOTE! These match bits 12..15 of stat.st_mode
 * (ie "(i_mode >> 12) & 15").
 */
#    define DT_UNKNOWN 0
#    define DT_FIFO 1
#    define DT_CHR 2
#    define DT_DIR 4
#    define DT_BLK 6
#    define DT_REG 8
#    define DT_LNK 10
#    define DT_SOCK 12
#    define DT_WHT 14

#    define OSYNC_METADATA (1 << 0)
#    define OSYNC_DATA (1 << 1)
#    define OSYNC_INODE (1 << 2)
int generic_osync_inode(struct inode *, struct address_space *, int);

/*
 * This is the "filldir" function type, used by readdir() to let
 * the kernel specify what kind of dirent layout it wants to have.
 * This allows the kernel to read directories into kernel space or
 * to have different dirent layouts depending on the binary type.
 */
typedef int (*filldir_t)(void *, const char *, int, loff_t, ino_t, unsigned);

/**
 * gendisk��fops�ֶ�ָ��һ����block_device_operations���ñ�Ϊ���豸��Ҫ��������˼������Ƶķ���
 */
struct block_device_operations {
    /**
	 * �򿪿��豸�ļ�
	 */
    int (*open)(struct inode *, struct file *);
    /**
	 * �رնԿ��豸�ļ������һ������
	 */
    int (*release)(struct inode *, struct file *);
    /**
	 * �ڿ��豸�ļ��Ϸ���ioctl()ϵͳ���ã�ʹ�ô��ں�����
	 * ����������ɿ��豸�㴦�������豸��ioctl��ʮ�ֶ�С��
	 */
    int (*ioctl)(struct inode *, struct file *, unsigned, unsigned long);
    /**
	 * �ڿ��豸�ļ��Ϸ���ioctl()ϵͳ���ã���ʹ�ô��ں�����
	 */
    long (*compat_ioctl)(struct file *, unsigned, unsigned long);
    /**
	 * �����ƶ������Ƿ��Ѿ������仯���������̣�������ֵΪ��0ֵ��ʾ�����ˡ�
	 */
    int (*media_changed)(struct gendisk *);
    /**
	 * �����豸�Ƿ������Ч���ݡ�
	 */
    int (*revalidate_disk)(struct gendisk *);
    struct module *owner;
};

/*
 * "descriptor" for what we're up to with a read for sendfile().
 * This allows us to use the same read code yet
 * have multiple different users of the data that
 * we read from a file.
 *
 * The simplest case just copies the data to user
 * mode.
 */
/**
 * ��ÿ���û�̬����������ص��ļ���������״̬��
 */
typedef struct {
    /**
	 * �Ѿ��������û�̬���������ֽ���
	 */
    size_t written;
    /**
	 * �����͵��ֽ���
	 */
    size_t count;
    /**
	 * ���û�̬�������еĵ�ǰλ��
	 */
    union {
        char __user *buf;
        void *data;
    } arg;
    /**
	 * �������Ĵ����롣0��ʾ�޴���
	 */
    int error;
} read_descriptor_t;

typedef int (*read_actor_t)(read_descriptor_t *, struct page *, unsigned long,
                            unsigned long);

/* These macros are for out of kernel modules to test that
 * the kernel supports the unlocked_ioctl and compat_ioctl
 * fields in struct file_operations. */
#    define HAVE_COMPAT_IOCTL 1
#    define HAVE_UNLOCKED_IOCTL 1

/*
 * NOTE:
 * read, write, poll, fsync, readv, writev, unlocked_ioctl and compat_ioctl
 * can be called without the big kernel lock held in all filesystems.
 */
/**
 * �ļ�����ѡ��
 */
struct file_operations {
    /**
	 * ӵ�иýṹ��ģ���ָ�롣����ģ�����ڱ�ʹ��ʱ����ж��ģ�顣
	 * ��������������£��ó�Ա���ᱻ��ʼ��ΪTHIS_MODULE��
	 */
    struct module *owner;
    /**
	 * ����llseek�����޸��ļ��ĵ�ǰ��дλ�á�������λ����Ϊ����ֵ���ء�
	 * ����loff_t��һ����ƫ��������ʹ��32λƽ̨��Ҳ����ռ��64λ�����ݿ��ȡ�
	 * ����ʱ����һ�����ķ���ֵ������������ָ����NULL����seek�ĵ��ý�����ĳ�ֲ���Ԥ�ڵķ�ʽ�޸�file�ṹ�е�λ�ü�����
	 */
    loff_t (*llseek)(struct file *, loff_t, int);
    /**
	 * �������豸�ж�ȡ���ݡ��ú���ָ�뱻��ΪNULLʱ��������readϵͳ���ó���������-EINVAL���������طǸ�ֵ��ʾ�ɹ���ȡ���ֽ�����
	 */
    ssize_t (*read)(struct file *, char __user *, size_t, loff_t *);
    /**
	 * ��ʼ��һ���첽�Ķ�ȡ���������ں�������֮ǰ���ܲ�����ɵĶ�ȡ����������÷���ΪNULL�����еĲ�����ͨ��readͬ����ɡ�
	 */
    ssize_t (*aio_read)(struct kiocb *, char __user *, size_t, loff_t);
    /**
	 * ���豸�������ݡ����û�����������writeϵͳ���û�����򷵻�һ��-EINVAL���������ֵ�Ǹ������ʾ�ɹ�д����ֽ�����
	 */
    ssize_t (*write)(struct file *, const char __user *, size_t, loff_t *);
    /**
	 * ��ʼ���豸�ϵ��첽д�������
	 */
    ssize_t (*aio_write)(struct kiocb *, const char __user *, size_t, loff_t);
    /**
	 * �����豸�ļ���˵������ֶ�Ӧ��ΪNULL���������ڶ�ȡĿ¼��ֻ���ļ�ϵͳ���á�
	 * filldir_t������ȡĿ¼��ĸ����ֶΡ�
	 */
    int (*readdir)(struct file *, void *, filldir_t);
    /**
	 * POLL������poll��epoll��select������ϵͳ���õĺ��ʵ�֡�������ϵͳ���ÿ�������ѯĳ�������ļ��������ϵĶ�ȡ��д���Ƿ�ᱻ������
	 * poll����Ӧ�÷���һ��λ���룬����ָ���������Ķ�ȡ��д���Ƿ���ܡ�����Ҳ�����ں��ṩ�����ý�����������״ֱ̬��IO��Ϊ����ʱ����Ϣ��
	 * �����������POLL��������ΪNULL�����豸�ᱻ��Ϊ�ȿɶ�Ҳ��д�����Ҳ���������
	 */
    unsigned int (*poll)(struct file *, struct poll_table_struct *);
    /**
	 * ϵͳ����ioctl�ṩ��һ��ִ���豸��������ķ���(���ʽ�����̵�ĳ���ŵ�����Ȳ��Ƕ�Ҳ����д����)��
	 * ���⣬�ں˻���ʶ��һ����ioctl��������ص���fops���е�ioctl������豸���ṩioctl��ڵ㣬������κ��ں�δԤ�ȶ��������ioctlϵͳ���ý����ش���(-ENOTYY)
	 */
    int (*ioctl)(struct inode *, struct file *, unsigned int, unsigned long);
    /**
	 * ��ioctl���ƣ����ǲ���ȡ���ں�����
	 */
    long (*unlocked_ioctl)(struct file *, unsigned int, unsigned long);
    /**
	 * 64λ�ں�ʹ�ø÷���ʵ��32λϵͳ���á�
	 */
    long (*compat_ioctl)(struct file *, unsigned int, unsigned long);
    /**
	 * �����豸�ڴ�ӳ�䵽���̵�ַ�ռ䡣����豸û��ʵ�������������ômmapϵͳ���ý�����-ENODEV��
	 */
    int (*mmap)(struct file *, struct vm_area_struct *);
    /**
	 * ������ʼ���Ƕ��豸�ļ�ִ�еĵ�һ��������Ȼ��ȴ����Ҫ����������һ��Ҫ����һ����Ӧ�ķ�����
	 * ���������ΪNULL���豸�Ĵ򿪲�����Զ�ɹ�����ϵͳ����֪ͨ��������
	 */
    int (*open)(struct inode *, struct file *);
    /**
	 * ��flush�����ĵ��÷����ڽ��̹ر��豸�ļ�������������ʱ����Ӧ��ִ��(���ȴ�)�豸����δ���Ĳ�����
	 * �벻Ҫ����ͬ�û�����ʹ�õ�fsync�����������Ŀǰ��flush�����������������������򡣱��磬SCSI�Ŵ���������������ȷ���豸���ر�֮ǰ���е����ݶ���д��Ŵ��С�
	 * ���flush����ΪNULL���ں˽��򵥵غ����û�Ӧ�ó��������
	 */
    int (*flush)(struct file *);
    /**
	 * ��file�ṹ���ͷ�ʱ�������������������open���ƣ�Ҳ���Խ�release����ΪNULL��
	 */
    int (*release)(struct inode *, struct file *);
    /**
	 * �÷�����fsyncϵͳ���õĺ��ʵ�֡��û���������ˢ�´����������ݡ������������û��ʵ����һ������fsyncϵͳ���ý�����-EINVAL��
	 */
    int (*fsync)(struct file *, struct dentry *, int datasync);
    /**
	 * ����fsync���첽�汾��
	 */
    int (*aio_fsync)(struct kiocb *, int datasync);
    /**
	 * �����������֪ͨ�豸��FASYNC��־�����˱仯���첽֪ͨ�ǱȽϸ߼��Ļ��⣬����豸��֧���첽֪ͨ�����ֶο�����NULL��
	 */
    int (*fasync)(int, struct file *, int);
    /**
	 * LOCK��������ʵ���ļ������������ǳ����ļ�����ȱ�ٵ����ԡ������豸�������򼸺���������ʵ�����������
	 */
    int (*lock)(struct file *, int, struct file_lock *);
    /**
	 * ��writev����ʵ�ַ�ɢ���ۼ��͵Ķ�д������Ӧ�ó�����ʱ��Ҫ�����漰����ڴ�����ĵ��ζ���д������
	 * ������Щϵͳ���ÿ�������๤����������ǿ�Ӷ�������ݿ������������������ΪNULL���ͻ����read��write����(�����Ƕ��)
	 */
    ssize_t (*readv)(struct file *, const struct iovec *, unsigned long,
                     loff_t *);
    ssize_t (*writev)(struct file *, const struct iovec *, unsigned long,
                      loff_t *);
    /**
	 * �������ʵ��sendfileϵͳ���õĶ�ȡ���֡�sendfileϵͳ��������С�ĸ��Ʋ��������ݴ�һ���ļ��������ƶ�����һ����
	 * ���磬WEB������������������������޸��ļ������ݷ��͵��������ӡ��豸��������ͨ����sendfile����ΪNULL��
	 */
    ssize_t (*sendfile)(struct file *, loff_t *, size_t, read_actor_t, void *);
    /**
	 * sendpage��sendfileϵͳ���õ���һ�롣�����ں˵����Խ����ݷ��͵���Ӧ���ļ���ÿ��һ������ҳ��
	 * �豸��������ͨ��Ҳ����Ҫʵ��sendfile��
	 */
    ssize_t (*sendpage)(struct file *, struct page *, int, size_t, loff_t *,
                        int);
    /**
	 * �ڽ��̵ĵ�ַ�ռ����ҵ�һ�����ʵ�λ�ã��Ա㽫�ײ��豸�е��ڴ��ӳ�䵽��λ�á�
	 * ������ͨ�����ڴ����������ɣ����÷����Ĵ��ڿ�������������ǿ�������ض��豸��Ҫ���κζ���Ҫ�󡣴󲿷�������������ø÷���ΪNULL��
	 */
    unsigned long (*get_unmapped_area)(struct file *, unsigned long,
                                       unsigned long, unsigned long,
                                       unsigned long);
    /**
	 * �÷�������ģ���鴫�ݸ�fcntl���õı�־����ǰֻ������NFS
	 */
    int (*check_flags)(int);
    /**
	 * ��Ӧ�ó���ʹ��fcntl������Ŀ¼�ı�֪ͨʱ���÷����������á��÷��������ļ�ϵͳ���ã��������򲻱�ʵ��dir_notify��
	 * ��ǰ������CIFS��
	 */
    int (*dir_notify)(struct file *filp, unsigned long arg);
    /**
	 * ���ڶ���flockϵͳ���õ���Ϊ����������ͼ���ļ�����ʱ���ص��˺�����
	 */
    int (*flock)(struct file *, int, struct file_lock *);
};

/**
 * �����ڵ����
 */
struct inode_operations {
    /**
	 * ��ĳһĿ¼�£�Ϊ��Ŀ¼�������ص���ͨ�ļ�����һ���µĴ��������ڵ㡣
	 */
    int (*create)(struct inode *, struct dentry *, int, struct nameidata *);
    /**
	 * Ϊ������һ��Ŀ¼������е��ļ�����Ӧ�������ڵ����Ŀ¼?
	 */
    struct dentry *(*lookup)(struct inode *, struct dentry *,
                             struct nameidata *);
    /**
	 * ����Ӳ���ӡ�
	 */
    int (*link)(struct dentry *, struct inode *, struct dentry *);
    /**
	 * ɾ���ļ���Ӳ���ӡ�
	 */
    int (*unlink)(struct inode *, struct dentry *);
    /**
	 * ���������ӡ�
	 */
    int (*symlink)(struct inode *, struct dentry *, const char *);
    /**
	 * ����Ŀ¼��
	 */
    int (*mkdir)(struct inode *, struct dentry *, int);
    /**
	 * �Ƴ�Ŀ¼��
	 */
    int (*rmdir)(struct inode *, struct dentry *);
    /**
	 * Ϊ�ض��豸�ļ�����һ�������ڵ㡣
	 */
    int (*mknod)(struct inode *, struct dentry *, int, dev_t);
    /**
	 * �������ļ���
	 */
    int (*rename)(struct inode *, struct dentry *, struct inode *,
                  struct dentry *);
    /**
	 * ��ȡ�������Ӷ�Ӧ���ļ�·������
	 */
    int (*readlink)(struct dentry *, char __user *, int);
    /**
	 * ���������ڵ����ָ���ķ���������
	 */
    int (*follow_link)(struct dentry *, struct nameidata *);
    /**
	 * �ͷ�follow_link�������ʱ���ݽṹ��
	 */
    void (*put_link)(struct dentry *, struct nameidata *);
    /**
	 * ����i_size�ֶΣ��޸������ڵ���ص��ļ����ȡ�
	 */
    void (*truncate)(struct inode *);
    /**
	 * ����ļ�Ȩ�ޡ�
	 */
    int (*permission)(struct inode *, int, struct nameidata *);
    /**
	 * �޸��ļ����ԡ�
	 */
    int (*setattr)(struct dentry *, struct iattr *);
    /**
	 * ��ȡ�ļ����ԡ�
	 */
    int (*getattr)(struct vfsmount *mnt, struct dentry *, struct kstat *);
    /**
	 * ������չ���ԡ���Щ���Է��������ڵ���Ĵ��̿��С�
	 */
    int (*setxattr)(struct dentry *, const char *, const void *, size_t, int);
    /**
	 * ��ȡ��չ���ԡ�
	 */
    ssize_t (*getxattr)(struct dentry *, const char *, void *, size_t);
    /**
	 * ��ȡ��չ�������Ƶ�����������
	 */
    ssize_t (*listxattr)(struct dentry *, char *, size_t);
    /**
	 * ɾ�������ڵ����չ���ԡ�
	 */
    int (*removexattr)(struct dentry *, const char *);
};

struct seq_file;

extern ssize_t vfs_read(struct file *, char __user *, size_t, loff_t *);
extern ssize_t vfs_write(struct file *, const char __user *, size_t, loff_t *);
extern ssize_t vfs_readv(struct file *, const struct iovec __user *,
                         unsigned long, loff_t *);
extern ssize_t vfs_writev(struct file *, const struct iovec __user *,
                          unsigned long, loff_t *);

/*
 * NOTE: write_inode, delete_inode, clear_inode, put_inode can be called
 * without the big kernel lock held in all filesystems.
 */
/**
 * �������������
 */
struct super_operations {
    /**
	 * Ϊ�����ڵ�������ռ䣬���������ļ�ϵͳ����������Ҫ�Ŀռ䡣
	 */
    struct inode *(*alloc_inode)(struct super_block *sb);
    /**
	 * �ͷ������ڵ����
	 */
    void (*destroy_inode)(struct inode *);

    /**
	 * �ô����ϵ�������������ڵ������ֶΡ�
	 * �����ڵ�����i_ino�ֶα�ʶ�Ӵ�����Ҫ�����ڵ㡣
	 */
    void (*read_inode)(struct inode *);

    /**
  	 * �������ڵ���Ϊ��ʱ���á���־�ļ�ϵͳ�������´����ϵ��ļ�ϵͳ��־��
  	 */
    void (*dirty_inode)(struct inode *);
    /**
	 * ���������ڵ��������ݡ�flag������ʾIO�����Ƿ�Ӧ��ͬ����
	 */
    int (*write_inode)(struct inode *, int);
    /**
	 * ���������ڵ�����ü���ֵ��
	 */
    void (*put_inode)(struct inode *);
    /**
	 * �����һ���û��ͷ������ڵ�ʱ���á�ͨ������generic_drop_inode��
	 */
    void (*drop_inode)(struct inode *);
    /**
	 * ɾ���ڴ��е������ڵ�ʹ����ϵ��ļ����ݺ�Ԫ���ݡ�
	 */
    void (*delete_inode)(struct inode *);
    /**
	 * �����ļ�ϵͳ��ж�ض��ͷŶԳ���������á�
	 */
    void (*put_super)(struct super_block *);
    /**
	 * �����ļ�ϵͳ�����顣
	 */
    void (*write_super)(struct super_block *);
    /**
	 * ����ļ�ϵͳ�Ը��´������ļ�ϵͳ���ݽṹ
	 */
    int (*sync_fs)(struct super_block *sb, int wait);
    /**
	 * �������ļ�ϵͳ���޸ģ�����ָ����������ݸ��³����顣
	 * ���ļ�ϵͳ������ʱ���á�����LVM���������
	 */
    void (*write_super_lockfs)(struct super_block *);
    /**
	 * ȡ����write_super_lockfs������
	 */
    void (*unlockfs)(struct super_block *);
    /**
	 * �����ļ�ϵͳ��ͳ����Ϣ��
	 */
    int (*statfs)(struct super_block *, struct kstatfs *);
    /**
	 * ���µ�ѡ�����°�װ�ļ�ϵͳ��
	 */
    int (*remount_fs)(struct super_block *, int *, char *);
    /**
	 * �������������ڵ�ʱ���á�
	 */
    void (*clear_inode)(struct inode *);
    /**
	 * ��ʼж�ز�����ֻ��NFS��ʹ�á�
	 */
    void (*umount_begin)(struct super_block *);

    /**
	 * ��ʾ�ض��ļ�ϵͳ��ѡ�
	 */
    int (*show_options)(struct seq_file *, struct vfsmount *);

    /**
	 * ��ȡ�޶����á�
	 */
    ssize_t (*quota_read)(struct super_block *, int, char *, size_t, loff_t);
    /**
	 * �޸��޶����á�
	 */
    ssize_t (*quota_write)(struct super_block *, int, const char *, size_t,
                           loff_t);
};

/* Inode state bits.  Protected by inode_lock. */
#    define I_DIRTY_SYNC 1     /* Not dirty enough for O_DATASYNC */
#    define I_DIRTY_DATASYNC 2 /* Data-related inode changes pending */
#    define I_DIRTY_PAGES 4    /* Data-related inode changes pending */
#    define __I_LOCK 3
/**
 * �����ڵ������IO�����С�
 */
#    define I_LOCK (1 << __I_LOCK)
/**
 * �����ڵ����ڱ��ͷš�
 */
#    define I_FREEING 16
/**
 * �����ڵ��������ݲ��������塣
 */
#    define I_CLEAR 32
/**
 * �����ڵ�����Ѿ����䣬���ǻ�û�дӴ����ж�ȡ���ݡ�
 */
#    define I_NEW 64

/**
 * �ж������ڵ��Ƿ�Ϊ�ࡣ
 */
#    define I_DIRTY (I_DIRTY_SYNC | I_DIRTY_DATASYNC | I_DIRTY_PAGES)

extern void __mark_inode_dirty(struct inode *, int);
static inline void mark_inode_dirty(struct inode *inode)
{
    __mark_inode_dirty(inode, I_DIRTY);
}

static inline void mark_inode_dirty_sync(struct inode *inode)
{
    __mark_inode_dirty(inode, I_DIRTY_SYNC);
}

static inline void touch_atime(struct vfsmount *mnt, struct dentry *dentry)
{
    /* per-mountpoint checks will go here */
    update_atime(dentry->d_inode);
}

static inline void file_accessed(struct file *file)
{
    if(!(file->f_flags & O_NOATIME))
        touch_atime(file->f_vfsmnt, file->f_dentry);
}

int sync_inode(struct inode *inode, struct writeback_control *wbc);

/**
 * &export_operations - for nfsd to communicate with file systems
 * decode_fh:      decode a file handle fragment and return a &struct dentry
 * encode_fh:      encode a file handle fragment from a dentry
 * get_name:       find the name for a given inode in a given directory
 * get_parent:     find the parent of a given directory
 * get_dentry:     find a dentry for the inode given a file handle sub-fragment
 *
 * Description:
 *    The export_operations structure provides a means for nfsd to communicate
 *    with a particular exported file system  - particularly enabling nfsd and
 *    the filesystem to co-operate when dealing with file handles.
 *
 *    export_operations contains two basic operation for dealing with file handles,
 *    decode_fh() and encode_fh(), and allows for some other operations to be defined
 *    which standard helper routines use to get specific information from the
 *    filesystem.
 *
 *    nfsd encodes information use to determine which filesystem a filehandle
 *    applies to in the initial part of the file handle.  The remainder, termed a
 *    file handle fragment, is controlled completely by the filesystem.
 *    The standard helper routines assume that this fragment will contain one or two
 *    sub-fragments, one which identifies the file, and one which may be used to
 *    identify the (a) directory containing the file.
 *
 *    In some situations, nfsd needs to get a dentry which is connected into a
 *    specific part of the file tree.  To allow for this, it passes the function
 *    acceptable() together with a @context which can be used to see if the dentry
 *    is acceptable.  As there can be multiple dentrys for a given file, the filesystem
 *    should check each one for acceptability before looking for the next.  As soon
 *    as an acceptable one is found, it should be returned.
 *
 * decode_fh:
 *    @decode_fh is given a &struct super_block (@sb), a file handle fragment (@fh, @fh_len)
 *    and an acceptability testing function (@acceptable, @context).  It should return
 *    a &struct dentry which refers to the same file that the file handle fragment refers
 *    to,  and which passes the acceptability test.  If it cannot, it should return
 *    a %NULL pointer if the file was found but no acceptable &dentries were available, or
 *    a %ERR_PTR error code indicating why it couldn't be found (e.g. %ENOENT or %ENOMEM).
 *
 * encode_fh:
 *    @encode_fh should store in the file handle fragment @fh (using at most @max_len bytes)
 *    information that can be used by @decode_fh to recover the file refered to by the
 *    &struct dentry @de.  If the @connectable flag is set, the encode_fh() should store
 *    sufficient information so that a good attempt can be made to find not only
 *    the file but also it's place in the filesystem.   This typically means storing
 *    a reference to de->d_parent in the filehandle fragment.
 *    encode_fh() should return the number of bytes stored or a negative error code
 *    such as %-ENOSPC
 *
 * get_name:
 *    @get_name should find a name for the given @child in the given @parent directory.
 *    The name should be stored in the @name (with the understanding that it is already
 *    pointing to a a %NAME_MAX+1 sized buffer.   get_name() should return %0 on success,
 *    a negative error code or error.
 *    @get_name will be called without @parent->i_sem held.
 *
 * get_parent:
 *    @get_parent should find the parent directory for the given @child which is also
 *    a directory.  In the event that it cannot be found, or storage space cannot be
 *    allocated, a %ERR_PTR should be returned.
 *
 * get_dentry:
 *    Given a &super_block (@sb) and a pointer to a file-system specific inode identifier,
 *    possibly an inode number, (@inump) get_dentry() should find the identified inode and
 *    return a dentry for that inode.
 *    Any suitable dentry can be returned including, if necessary, a new dentry created
 *    with d_alloc_root.  The caller can then find any other extant dentrys by following the
 *    d_alias links.  If a new dentry was created using d_alloc_root, DCACHE_NFSD_DISCONNECTED
 *    should be set, and the dentry should be d_rehash()ed.
 *
 *    If the inode cannot be found, either a %NULL pointer or an %ERR_PTR code can be returned.
 *    The @inump will be whatever was passed to nfsd_find_fh_dentry() in either the
 *    @obj or @parent parameters.
 *
 * Locking rules:
 *  get_parent is called with child->d_inode->i_sem down
 *  get_name is not (which is possibly inconsistent)
 */

struct export_operations {
    struct dentry *(*decode_fh)(
        struct super_block *sb, __u32 *fh, int fh_len, int fh_type,
        int (*acceptable)(void *context, struct dentry *de), void *context);
    int (*encode_fh)(struct dentry *de, __u32 *fh, int *max_len,
                     int connectable);

    /* the following are only called from the filesystem itself */
    int (*get_name)(struct dentry *parent, char *name, struct dentry *child);
    struct dentry *(*get_parent)(struct dentry *child);
    struct dentry *(*get_dentry)(struct super_block *sb, void *inump);

    /* This is set by the exporting module to a standard helper */
    struct dentry *(*find_exported_dentry)(
        struct super_block *sb, void *obj, void *parent,
        int (*acceptable)(void *context, struct dentry *de), void *context);
};

extern struct dentry *
find_exported_dentry(struct super_block *sb, void *obj, void *parent,
                     int (*acceptable)(void *context, struct dentry *de),
                     void *context);

/**
 * ���ں�֧�ֵ�ÿһ���ļ�ϵͳ������һ�������Ľṹ�������������
 */
struct file_system_type {
    /**
	 * �ļ�ϵͳ���͵����� 
	 */
    const char *name;
    /**
	 * ���ļ�ϵͳ���͵����� 
	 */
    int fs_flags;
    /**
	 * ����ָ�룬����װ�����͵��ļ�ϵͳʱ������VFS���ô����̴��豸�Ͻ����ļ�ϵͳ��superblock�����ڴ���
	 */
    struct super_block *(*get_sb)(struct file_system_type *, int, const char *,
                                  void *);
    /**
	 * ɾ��������ķ�����
	 */
    void (*kill_sb)(struct super_block *);
    /**
	 * ָ��ʵ���ļ�ϵͳ��ģ���ָ�롣
	 */
    struct module *owner;
    /**
	 * ��һ���ļ�ϵͳָ�롣
	 */
    struct file_system_type *next;
    /**
	 * ������ͬ�ļ�ϵͳ���͵ĳ��������������ͷ��
	 */
    struct list_head fs_supers;
};

struct super_block *
get_sb_bdev(struct file_system_type *fs_type, int flags, const char *dev_name,
            void *data, int (*fill_super)(struct super_block *, void *, int));
struct super_block *get_sb_single(
    struct file_system_type *fs_type, int flags, void *data,
    int (*fill_super)(struct super_block *, void *,
                      int)); // get_sb_single :获取超级块填充其信息，然后挂载
struct super_block *
get_sb_nodev(struct file_system_type *fs_type, int flags, void *data,
             int (*fill_super)(struct super_block *, void *, int));
void generic_shutdown_super(struct super_block *sb);
void kill_block_super(struct super_block *sb);
void kill_anon_super(struct super_block *sb);
void kill_litter_super(struct super_block *sb);
void deactivate_super(struct super_block *sb);
int set_anon_super(struct super_block *s, void *data);
struct super_block *sget(struct file_system_type *type,
                         int (*test)(struct super_block *, void *),
                         int (*set)(struct super_block *, void *), void *data);
struct super_block *get_sb_pseudo(struct file_system_type *, char *,
                                  struct super_operations *ops, unsigned long);
int __put_super(struct super_block *sb);
int __put_super_and_need_restart(struct super_block *sb);
void unnamed_dev_init(void);

/* Alas, no aliases. Too much hassle with bringing module.h everywhere */
#    define fops_get(fops)                                                     \
        (((fops) && try_module_get((fops)->owner) ? (fops) : NULL))
#    define fops_put(fops)                                                     \
        do {                                                                   \
            if(fops)                                                           \
                module_put((fops)->owner);                                     \
        } while(0)

extern int register_filesystem(struct file_system_type *);
extern int unregister_filesystem(struct file_system_type *);
extern struct vfsmount *kern_mount(struct file_system_type *);
extern int may_umount_tree(struct vfsmount *);
extern int may_umount(struct vfsmount *);
extern long do_mount(char *, char *, char *, unsigned long, void *);

extern int vfs_statfs(struct super_block *, struct kstatfs *);

#    define FLOCK_VERIFY_READ 1
#    define FLOCK_VERIFY_WRITE 2

extern int locks_mandatory_locked(struct inode *);
extern int locks_mandatory_area(int, struct inode *, struct file *, loff_t,
                                size_t);

/*
 * Candidates for mandatory locking have the setgid bit set
 * but no group execute bit -  an otherwise meaningless combination.
 */
#    define MANDATORY_LOCK(inode)                                              \
        (IS_MANDLOCK(inode)                                                    \
         && ((inode)->i_mode & (S_ISGID | S_IXGRP)) == S_ISGID)

static inline int locks_verify_locked(struct inode *inode)
{
    if(MANDATORY_LOCK(inode))
        return locks_mandatory_locked(inode);
    return 0;
}

extern int rw_verify_area(int, struct file *, loff_t *, size_t);

static inline int locks_verify_truncate(struct inode *inode, struct file *filp,
                                        loff_t size)
{
    if(inode->i_flock && MANDATORY_LOCK(inode))
        return locks_mandatory_area(FLOCK_VERIFY_WRITE, inode, filp,
                                    size < inode->i_size ? size : inode->i_size,
                                    (size < inode->i_size
                                         ? inode->i_size - size
                                         : size - inode->i_size));
    return 0;
}

static inline int break_lease(struct inode *inode, unsigned int mode)
{
    if(inode->i_flock)
        return __break_lease(inode, mode);
    return 0;
}

/* fs/open.c */

extern int do_truncate(struct dentry *, loff_t start);
extern struct file *filp_open(const char *, int, int);
extern struct file *dentry_open(struct dentry *, struct vfsmount *, int);
extern int filp_close(struct file *, fl_owner_t id);
extern char *getname(const char __user *);

/* fs/dcache.c */
extern void __init vfs_caches_init_early(void);
extern void __init vfs_caches_init(unsigned long);

#    define __getname() kmem_cache_alloc(names_cachep, SLAB_KERNEL)
#    define __putname(name) kmem_cache_free(names_cachep, (void *)(name))
#    ifndef CONFIG_AUDITSYSCALL
#        define putname(name) __putname(name)
#    else
#        define putname(name)                                                  \
            do {                                                               \
                if(unlikely(current->audit_context))                           \
                    audit_putname(name);                                       \
                else                                                           \
                    __putname(name);                                           \
            } while(0)
#    endif

extern int register_blkdev(unsigned int, const char *);
extern int unregister_blkdev(unsigned int, const char *);
extern struct block_device *bdget(dev_t);
extern void bd_set_size(struct block_device *, loff_t size);
extern void bd_forget(struct inode *inode);
extern void bdput(struct block_device *);
extern struct block_device *open_by_devnum(dev_t, unsigned);
extern struct file_operations def_blk_fops;
extern struct address_space_operations def_blk_aops;
extern struct file_operations def_chr_fops;
extern struct file_operations bad_sock_fops;
extern struct file_operations def_fifo_fops;
extern int ioctl_by_bdev(struct block_device *, unsigned, unsigned long);
extern int blkdev_ioctl(struct inode *, struct file *, unsigned, unsigned long);
extern long compat_blkdev_ioctl(struct file *, unsigned, unsigned long);
extern int blkdev_get(struct block_device *, mode_t, unsigned);
extern int blkdev_put(struct block_device *);
extern int bd_claim(struct block_device *, void *);
extern void bd_release(struct block_device *);

/* fs/char_dev.c */
extern int alloc_chrdev_region(dev_t *, unsigned, unsigned, const char *);
extern int register_chrdev_region(dev_t, unsigned, const char *);
extern int register_chrdev(unsigned int, const char *,
                           struct file_operations *);
extern int unregister_chrdev(unsigned int, const char *);
extern void unregister_chrdev_region(dev_t, unsigned);
extern int chrdev_open(struct inode *, struct file *);

/* fs/block_dev.c */
#    define BDEVNAME_SIZE 32 /* Largest string for a blockdev identifier */
extern const char *__bdevname(dev_t, char *buffer);
extern const char *bdevname(struct block_device *bdev, char *buffer);
extern struct block_device *lookup_bdev(const char *);
extern struct block_device *open_bdev_excl(const char *, int, void *);
extern void close_bdev_excl(struct block_device *);

extern void init_special_inode(struct inode *, umode_t, dev_t);

/* Invalid inode operations -- fs/bad_inode.c */
extern void make_bad_inode(struct inode *);
extern int is_bad_inode(struct inode *);

extern struct file_operations read_fifo_fops;
extern struct file_operations write_fifo_fops;
extern struct file_operations rdwr_fifo_fops;
extern struct file_operations read_pipe_fops;
extern struct file_operations write_pipe_fops;
extern struct file_operations rdwr_pipe_fops;

extern int fs_may_remount_ro(struct super_block *);

/*
 * return READ, READA, or WRITE
 */
#    define bio_rw(bio) ((bio)->bi_rw & (RW_MASK | RWA_MASK))

/*
 * return data direction, READ or WRITE
 */
/**
 * ȷ��һ��BIO������һ����������д����
 */
#    define bio_data_dir(bio) ((bio)->bi_rw & 1)

extern int check_disk_change(struct block_device *);
extern int invalidate_inodes(struct super_block *);
extern int __invalidate_device(struct block_device *, int);
extern int invalidate_partition(struct gendisk *, int);
unsigned long invalidate_mapping_pages(struct address_space *mapping,
                                       pgoff_t start, pgoff_t end);
unsigned long invalidate_inode_pages(struct address_space *mapping);
static inline void invalidate_remote_inode(struct inode *inode)
{
    if(S_ISREG(inode->i_mode) || S_ISDIR(inode->i_mode)
       || S_ISLNK(inode->i_mode))
        invalidate_inode_pages(inode->i_mapping);
}
extern int invalidate_inode_pages2(struct address_space *mapping);
extern int write_inode_now(struct inode *, int);
extern int filemap_fdatawrite(struct address_space *);
extern int filemap_flush(struct address_space *);
extern int filemap_fdatawait(struct address_space *);
extern int filemap_write_and_wait(struct address_space *mapping);
extern void sync_supers(void);
extern void sync_filesystems(int wait);
extern void emergency_sync(void);
extern void emergency_remount(void);
extern int do_remount_sb(struct super_block *sb, int flags, void *data,
                         int force);
extern sector_t bmap(struct inode *, sector_t);
extern int setattr_mask(unsigned int);
extern int notify_change(struct dentry *, struct iattr *);
extern int permission(struct inode *, int, struct nameidata *);
extern int generic_permission(struct inode *, int,
                              int (*check_acl)(struct inode *, int));

extern int get_write_access(struct inode *);
extern int deny_write_access(struct file *);
static inline void put_write_access(struct inode *inode)
{
    atomic_dec(&inode->i_writecount);
}
static inline void allow_write_access(struct file *file)
{
    if(file)
        atomic_inc(&file->f_dentry->d_inode->i_writecount);
}
extern int do_pipe(int *);

extern int open_namei(const char *, int, int, struct nameidata *);
extern int may_open(struct nameidata *, int, int);

extern int kernel_read(struct file *, unsigned long, char *, unsigned long);
extern struct file *open_exec(const char *);

/* fs/dcache.c -- generic fs support functions */
extern int is_subdir(struct dentry *, struct dentry *);
extern ino_t find_inode_number(struct dentry *, struct qstr *);

#    include <linux/err.h>

/* needed for stackable file system support */
extern loff_t default_llseek(struct file *file, loff_t offset, int origin);

extern loff_t vfs_llseek(struct file *file, loff_t offset, int origin);

extern void inode_init_once(struct inode *);
extern void iput(struct inode *);
extern struct inode *igrab(struct inode *);
extern ino_t iunique(struct super_block *, ino_t);
extern int inode_needs_sync(struct inode *inode);
extern void generic_delete_inode(struct inode *inode);

extern struct inode *ilookup5(struct super_block *sb, unsigned long hashval,
                              int (*test)(struct inode *, void *), void *data);
extern struct inode *ilookup(struct super_block *sb, unsigned long ino);

extern struct inode *iget5_locked(struct super_block *, unsigned long,
                                  int (*test)(struct inode *, void *),
                                  int (*set)(struct inode *, void *), void *);
extern struct inode *iget_locked(struct super_block *, unsigned long);
extern void unlock_new_inode(struct inode *);

static inline struct inode *iget(struct super_block *sb, unsigned long ino)
{
    struct inode *inode = iget_locked(sb, ino);

    if(inode && (inode->i_state & I_NEW)) {
        sb->s_op->read_inode(inode);
        unlock_new_inode(inode);
    }

    return inode;
}

extern void __iget(struct inode *inode);
extern void clear_inode(struct inode *);
extern void destroy_inode(struct inode *);
extern struct inode *new_inode(struct super_block *);
extern int remove_suid(struct dentry *);
extern void remove_dquot_ref(struct super_block *, int, struct list_head *);
extern struct semaphore iprune_sem;

extern void __insert_inode_hash(struct inode *, unsigned long hashval);
extern void remove_inode_hash(struct inode *);
static inline void insert_inode_hash(struct inode *inode)
{
    __insert_inode_hash(inode, inode->i_ino);
}

extern struct file *get_empty_filp(void);
extern void file_move(struct file *f, struct list_head *list);
extern void file_kill(struct file *f);
struct bio;
extern void submit_bio(int, struct bio *);
extern int bdev_read_only(struct block_device *);
extern int set_blocksize(struct block_device *, int);
extern int sb_set_blocksize(struct super_block *, int);
extern int sb_min_blocksize(struct super_block *, int);

extern int generic_file_mmap(struct file *, struct vm_area_struct *);
extern int generic_file_readonly_mmap(struct file *, struct vm_area_struct *);
extern int file_read_actor(read_descriptor_t *desc, struct page *page,
                           unsigned long offset, unsigned long size);
extern int file_send_actor(read_descriptor_t *desc, struct page *page,
                           unsigned long offset, unsigned long size);
extern ssize_t generic_file_read(struct file *, char __user *, size_t,
                                 loff_t *);
int generic_write_checks(struct file *file, loff_t *pos, size_t *count,
                         int isblk);
extern ssize_t generic_file_write(struct file *, const char __user *, size_t,
                                  loff_t *);
extern ssize_t generic_file_aio_read(struct kiocb *, char __user *, size_t,
                                     loff_t);
extern ssize_t __generic_file_aio_read(struct kiocb *, const struct iovec *,
                                       unsigned long, loff_t *);
extern ssize_t generic_file_aio_write(struct kiocb *, const char __user *,
                                      size_t, loff_t);
extern ssize_t generic_file_aio_write_nolock(struct kiocb *,
                                             const struct iovec *,
                                             unsigned long, loff_t *);
extern ssize_t generic_file_direct_write(struct kiocb *, const struct iovec *,
                                         unsigned long *, loff_t, loff_t *,
                                         size_t, size_t);
extern ssize_t generic_file_buffered_write(struct kiocb *, const struct iovec *,
                                           unsigned long, loff_t, loff_t *,
                                           size_t, ssize_t);
extern ssize_t do_sync_read(struct file *filp, char __user *buf, size_t len,
                            loff_t *ppos);
extern ssize_t do_sync_write(struct file *filp, const char __user *buf,
                             size_t len, loff_t *ppos);
ssize_t generic_file_write_nolock(struct file *file, const struct iovec *iov,
                                  unsigned long nr_segs, loff_t *ppos);
extern ssize_t generic_file_sendfile(struct file *, loff_t *, size_t,
                                     read_actor_t, void *);
extern void do_generic_mapping_read(struct address_space *mapping,
                                    struct file_ra_state *, struct file *,
                                    loff_t *, read_descriptor_t *,
                                    read_actor_t);
extern void file_ra_state_init(struct file_ra_state *ra,
                               struct address_space *mapping);
extern ssize_t generic_file_direct_IO(int rw, struct kiocb *iocb,
                                      const struct iovec *iov, loff_t offset,
                                      unsigned long nr_segs);
extern ssize_t generic_file_readv(struct file *filp, const struct iovec *iov,
                                  unsigned long nr_segs, loff_t *ppos);
ssize_t generic_file_writev(struct file *filp, const struct iovec *iov,
                            unsigned long nr_segs, loff_t *ppos);
extern loff_t no_llseek(struct file *file, loff_t offset, int origin);
extern loff_t generic_file_llseek(struct file *file, loff_t offset, int origin);
extern loff_t remote_llseek(struct file *file, loff_t offset, int origin);
extern int generic_file_open(struct inode *inode, struct file *filp);
extern int nonseekable_open(struct inode *inode, struct file *filp);

/**
 * �Ӵ��̶����������ҳ,�������Ǹ��Ƶ��û�̬������.
 */
static inline void do_generic_file_read(struct file *filp, loff_t *ppos,
                                        read_descriptor_t *desc,
                                        read_actor_t actor)
{
    /**
	 * Ҫ��ȡ���ļ���Ӧ��address_space��������filp->f_mapping
	 */
    do_generic_mapping_read(filp->f_mapping, &filp->f_ra, filp, ppos, desc,
                            actor);
}

ssize_t __blockdev_direct_IO(int rw, struct kiocb *iocb, struct inode *inode,
                             struct block_device *bdev, const struct iovec *iov,
                             loff_t offset, unsigned long nr_segs,
                             get_blocks_t get_blocks, dio_iodone_t end_io,
                             int lock_type);

enum {
    DIO_LOCKING = 1, /* need locking between buffered and direct access */
    DIO_NO_LOCKING,  /* bdev; no locking at all between buffered/direct */
    DIO_OWN_LOCKING, /* filesystem locks buffered and direct internally */
};

static inline ssize_t
blockdev_direct_IO(int rw, struct kiocb *iocb, struct inode *inode,
                   struct block_device *bdev, const struct iovec *iov,
                   loff_t offset, unsigned long nr_segs,
                   get_blocks_t get_blocks, dio_iodone_t end_io)
{
    return __blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset, nr_segs,
                                get_blocks, end_io, DIO_LOCKING);
}

static inline ssize_t blockdev_direct_IO_no_locking(
    int rw, struct kiocb *iocb, struct inode *inode, struct block_device *bdev,
    const struct iovec *iov, loff_t offset, unsigned long nr_segs,
    get_blocks_t get_blocks, dio_iodone_t end_io)
{
    return __blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset, nr_segs,
                                get_blocks, end_io, DIO_NO_LOCKING);
}

static inline ssize_t blockdev_direct_IO_own_locking(
    int rw, struct kiocb *iocb, struct inode *inode, struct block_device *bdev,
    const struct iovec *iov, loff_t offset, unsigned long nr_segs,
    get_blocks_t get_blocks, dio_iodone_t end_io)
{
    return __blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset, nr_segs,
                                get_blocks, end_io, DIO_OWN_LOCKING);
}

extern struct file_operations generic_ro_fops;

#    define special_file(m)                                                    \
        (S_ISCHR(m) || S_ISBLK(m) || S_ISFIFO(m) || S_ISSOCK(m))

extern int vfs_readlink(struct dentry *, char __user *, int, const char *);
extern int vfs_follow_link(struct nameidata *, const char *);
extern int page_readlink(struct dentry *, char __user *, int);
extern int page_follow_link_light(struct dentry *, struct nameidata *);
extern void page_put_link(struct dentry *, struct nameidata *);
extern int page_symlink(struct inode *inode, const char *symname, int len);
extern struct inode_operations page_symlink_inode_operations;
extern int generic_readlink(struct dentry *, char __user *, int);
extern void generic_fillattr(struct inode *, struct kstat *);
extern int vfs_getattr(struct vfsmount *, struct dentry *, struct kstat *);
void inode_add_bytes(struct inode *inode, loff_t bytes);
void inode_sub_bytes(struct inode *inode, loff_t bytes);
loff_t inode_get_bytes(struct inode *inode);
void inode_set_bytes(struct inode *inode, loff_t bytes);

extern int vfs_readdir(struct file *, filldir_t, void *);

extern int vfs_stat(char __user *, struct kstat *);
extern int vfs_lstat(char __user *, struct kstat *);
extern int vfs_fstat(unsigned int, struct kstat *);

extern int vfs_ioctl(struct file *, unsigned int, unsigned int, unsigned long);

extern struct file_system_type *get_fs_type(const char *name);
extern struct super_block *get_super(struct block_device *);
extern struct super_block *user_get_super(dev_t);
extern void drop_super(struct super_block *sb);

extern int dcache_dir_open(struct inode *, struct file *);
extern int dcache_dir_close(struct inode *, struct file *);
extern loff_t dcache_dir_lseek(struct file *, loff_t, int);
extern int dcache_readdir(struct file *, void *, filldir_t);
extern int simple_getattr(struct vfsmount *, struct dentry *, struct kstat *);
extern int simple_statfs(struct super_block *, struct kstatfs *);
extern int simple_link(struct dentry *, struct inode *, struct dentry *);
extern int simple_unlink(struct inode *, struct dentry *);
extern int simple_rmdir(struct inode *, struct dentry *);
extern int simple_rename(struct inode *, struct dentry *, struct inode *,
                         struct dentry *);
extern int simple_sync_file(struct file *, struct dentry *, int);
extern int simple_empty(struct dentry *);
extern int simple_readpage(struct file *file, struct page *page);
extern int simple_prepare_write(struct file *file, struct page *page,
                                unsigned offset, unsigned to);
extern int simple_commit_write(struct file *file, struct page *page,
                               unsigned offset, unsigned to);

extern struct dentry *simple_lookup(struct inode *, struct dentry *,
                                    struct nameidata *);
extern ssize_t generic_read_dir(struct file *, char __user *, size_t, loff_t *);
extern struct file_operations simple_dir_operations;
extern struct inode_operations simple_dir_inode_operations;
struct tree_descr {
    char *name;
    struct file_operations *ops;
    int mode;
};
struct dentry *d_alloc_name(struct dentry *, const char *);
extern int simple_fill_super(struct super_block *, int, struct tree_descr *);
extern int simple_pin_fs(char *name, struct vfsmount **mount, int *count);
extern void simple_release_fs(struct vfsmount **mount, int *count);

extern ssize_t simple_read_from_buffer(void __user *, size_t, loff_t *,
                                       const void *, size_t);

extern int inode_change_ok(struct inode *, struct iattr *);
extern int __must_check inode_setattr(struct inode *, struct iattr *);

extern void inode_update_time(struct inode *inode, int ctime_too);

static inline ino_t parent_ino(struct dentry *dentry)
{
    ino_t res;

    spin_lock(&dentry->d_lock);
    res = dentry->d_parent->d_inode->i_ino;
    spin_unlock(&dentry->d_lock);
    return res;
}

/* kernel/fork.c */
extern int unshare_files(void);

/* Transaction based IO helpers */

/*
 * An argresp is stored in an allocated page and holds the
 * size of the argument or response, along with its content
 */
struct simple_transaction_argresp {
    ssize_t size;
    char data[0];
};

#    define SIMPLE_TRANSACTION_LIMIT                                           \
        (PAGE_SIZE - sizeof(struct simple_transaction_argresp))

char *simple_transaction_get(struct file *file, const char __user *buf,
                             size_t size);
ssize_t simple_transaction_read(struct file *file, char __user *buf,
                                size_t size, loff_t *pos);
int simple_transaction_release(struct inode *inode, struct file *file);

static inline void simple_transaction_set(struct file *file, size_t n)
{
    struct simple_transaction_argresp *ar = file->private_data;

    BUG_ON(n > SIMPLE_TRANSACTION_LIMIT);

    /*
	 * The barrier ensures that ar->size will really remain zero until
	 * ar->data is ready for reading.
	 */
    smp_mb();
    ar->size = n;
}

#    ifdef CONFIG_SECURITY
static inline char *alloc_secdata(void)
{
    return (char *)get_zeroed_page(GFP_KERNEL);
}

static inline void free_secdata(void *secdata)
{
    free_page((unsigned long)secdata);
}
#    else
static inline char *alloc_secdata(void)
{
    return (char *)1;
}

static inline void free_secdata(void *secdata) {}
#    endif /* CONFIG_SECURITY */

#endif /* __KERNEL__ */
#endif /* _LINUX_FS_H */
