/*
 * Ioctl definitions
 */

/* Use 'k' as magic number */
#define TW2823_IOC_MAGIC  'k'
/* Please use a different 8-bit number in your code */

#define TW2823_IOC_RESET        _IO(TW2823_IOC_MAGIC,   0)
#define TW2823_IOC_SETWINDOW    _IOW(TW2823_IOC_MAGIC,  1, int)
// #define TW2823_IOC_GQUANTUM _IOR(TW2823_IOC_MAGIC,  2, int)
// #define TW2823_IOC_SQUANTUM _IOW(TW2823_IOC_MAGIC,  3, int)
/* ... more to come */




#define TW2823_IOC_MAXNR 2