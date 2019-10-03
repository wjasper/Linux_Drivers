/*
  program to convert .iic (USB FPGA file) to ascii.
*/

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

unsigned char reverse(unsigned char b)
{
  b = (b & 0xf0) >> 4 | (b & 0x0f) << 4;
  b = (b & 0xcc) >> 2 | (b & 0x33) << 2;
  b = (b & 0xaa) >> 1 | (b & 0x55) << 1;
  return b;
}

int main()
{
  int fd;
  ssize_t count;
  int total_cnt = 0;
  char buf[8];
  int i;

  fd = open("./USB_CTR.bin", O_RDONLY);
  printf("static unsigned char FPGA_data[] = {\n");
  while ( (count = read(fd, buf, 8)) != 0 ) {
    for (i = 0; i < count; i++) {
     //  buf[i] = reverse(buf[i]);     // bit reversal
      printf("  0x%.2hhx,", buf[i]);
      total_cnt++;
    }
    printf("\n");
  }
  printf("};\n");
  close(fd);
  return 0;
}

