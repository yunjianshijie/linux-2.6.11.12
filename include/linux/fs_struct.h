#ifndef _LINUX_FS_STRUCT_H
#define _LINUX_FS_STRUCT_H

struct dentry;
struct vfsmount;


struct fs_struct {
	
	atomic_t count;
	
	rwlock_t lock;
	
	int umask; //用户文件创建掩码 ，用于设置文件权限
    /**
	 * root			根目录 目录项
	 * pwd			当前工作目录，所有相对路径  目录
	 * altroot		指向备用根目录的目录项
	 */
    struct dentry * root, * pwd, * altroot;
	/**
	 * rootmnt		
	 * pwdmnt		
	 * altrootmnt	
	 */
	struct vfsmount * rootmnt, * pwdmnt, * altrootmnt;
};

#define INIT_FS {				\
	.count		= ATOMIC_INIT(1),	\
	.lock		= RW_LOCK_UNLOCKED,	\
	.umask		= 0022, \
}

extern void exit_fs(struct task_struct *);
extern void set_fs_altroot(void);
extern void set_fs_root(struct fs_struct *, struct vfsmount *, struct dentry *);
extern void set_fs_pwd(struct fs_struct *, struct vfsmount *, struct dentry *);
extern struct fs_struct *copy_fs_struct(struct fs_struct *);
extern void put_fs_struct(struct fs_struct *);

#endif /* _LINUX_FS_STRUCT_H */
