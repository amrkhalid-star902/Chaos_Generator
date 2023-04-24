#ifndef _AES_H_
#define _AES_H_

#include <stdint.h>
#include <stddef.h>

#define AES128 1

#define AES_BLOCKLEN  16 //block length in bytes as aes 128 key contain 128 bits
#define AES_keyExpSize 176

struct AES_ctx
{
    uint8_t RoundKey[AES_keyExpSize];
};


void AES_Init(struct AES_ctx* ctx, uint8_t * key);
void AES_Encrypt(const struct AES_ctx * ctx , uint8_t * buffer );



#endif // AES_H_
