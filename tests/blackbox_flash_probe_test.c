#include <assert.h>
#include <stdio.h>
#include "../src/storage/blackbox_flash.c"
static void probe(uint8_t maker,uint8_t type,uint8_t capacity,uint32_t mb)
{
    fake_id[0]=maker;fake_id[1]=type;fake_id[2]=capacity;
    erase_program_commands=0;blackbox_sd_init();
    assert(blackbox_sd_capacity_mb()==mb);
    assert(blackbox_sd_is_enabled()==(mb!=0));
    assert(erase_program_commands==0);
    if(mb){assert(BANK_BYTES==mb*1024U*1024U/2);assert(bank_address(1)==BANK_BYTES);}
}
int main(void)
{
    probe(0xef,0x40,0x18,16); /* W25Q128 */
    probe(0x20,0x20,0x15,2);  /* M25P16 */
    probe(0xef,0xaa,0x21,0);  /* W25N01G: reject NAND */
    probe(0xef,0xaa,0x18,0);  /* same capacity byte cannot make NAND into NOR */
    probe(0xef,0x40,0x19,0);  /* unsupported 4-byte addressing */
    probe(0xff,0xff,0xff,0);  /* absent */
    puts("NOR capacity/bank layout and NAND rejection tests passed");
}
