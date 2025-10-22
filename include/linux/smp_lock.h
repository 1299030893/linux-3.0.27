#ifndef __LINUX_SMPLOCK_H
#define __LINUX_SMPLOCK_H


/* provoke build bug if not set */
#define lock_kernel()
#define unlock_kernel()
#define cycle_kernel_lock()			do { } while(0)

#define release_kernel_lock(task)		do { } while(0)
#define reacquire_kernel_lock(task)		0


#endif /* __LINUX_SMPLOCK_H */
