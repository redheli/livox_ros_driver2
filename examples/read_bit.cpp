
#include <stdint.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <map>


uint8_t *get_bits(uint8_t n, int bitswanted){
  uint8_t *bits = (uint8_t *)malloc(sizeof(int) * bitswanted);

  int k;
  for(k=0; k<bitswanted; k++){
    int mask =  1 << k;
    int masked_n = n & mask;
    int thebit = masked_n >> k;
    bits[k] = thebit;
  }

  return bits;
}

int main(){
  uint8_t n=12;

  // int  bitswanted = 2;

  // get bit 0 and bit 1
  auto bit01 = n & 0x03;
  printf("bit01 %d \n", bit01);

  if(bit01 == 0x00){
    printf("find it\n");
  }
  else{
    printf("not 0x02\n");
  }

  // get bit 2 and bit 3
  auto bit23 = (n >> 2) & 0x03;
  printf("bit23 %d \n", bit23);

  // uint8_t *bits = get_bits(n, bitswanted);

  // printf("%d = ", n);

  // int i;
  // for(i=bitswanted-1; i>=0;i--){
  //   printf("%d ", bits[i]);
  // }

  // printf("\n");
}