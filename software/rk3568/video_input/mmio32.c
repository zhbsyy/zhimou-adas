/* Native BAR1 descriptor submission. No memset, memcpy or register read.
 * Only the caller's already mapped BAR is accessed, at the three fixed offsets.
 */
#include <stdint.h>

#if defined(MMIO_TEST_HOST)
/* Ordinary-RAM unit tests only. Never a supported board backend. */
# define ORDER_WRITES() __asm__ __volatile__("" ::: "memory")
# if defined(_WIN32)
#  define EXPORT __declspec(dllexport)
# else
#  define EXPORT
# endif
#elif defined(__aarch64__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
# define ORDER_WRITES() __asm__ __volatile__("dmb oshst" ::: "memory")
# define EXPORT
#else
# error "Board helper requires little-endian AArch64"
#endif

EXPORT void mmio_submit_row(void *mapping, uint32_t command, uint64_t target)
{
    volatile uint8_t *bar = (volatile uint8_t *)mapping;
    ORDER_WRITES();
    *(volatile uint32_t *)(bar + 0x100) = command;
    ORDER_WRITES();
    *(volatile uint32_t *)(bar + 0x110) = (uint32_t)target;
    ORDER_WRITES();
    *(volatile uint32_t *)(bar + 0x120) = (uint32_t)(target >> 32);
    ORDER_WRITES();
}
