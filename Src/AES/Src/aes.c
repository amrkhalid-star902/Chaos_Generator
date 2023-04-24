#include "aes.h"


//Number of coloumns
#define NC 4

//Number of rounds
#define NR 10

#define getSBoxValue(num) (sbox[(num)])

// holding intermediate results
typedef uint8_t state_t[4][4];

//some function declerations
static void RotateWord(uint8_t * word);
static void GetSubWord(uint8_t * word);


// SBOX

static const uint8_t sbox[256] = {
  //0     1    2      3     4    5     6     7      8    9     A      B    C     D     E     F
  0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
  0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
  0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
  0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
  0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
  0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
  0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
  0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
  0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
  0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
  0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
  0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
  0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
  0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
  0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
  0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16 };


  // Round constant array
  static const uint8_t Rcon[11] = {
  0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36 };

// This function produces NC(NR+1) round keys.
static void KeyExpansion(uint8_t * RoundKey , const uint8_t * Key){

    unsigned i , j , k;
    uint8_t tempa[4]; //Used for operations on column and rows
    /*
        The key expansion function works as follow :
            N is the length of key in 32-bit words in our case N = 4
            K0,K1,....KN-1 are the 32 bit words of the original key
            R is the number of round keys needed 11 round keys in case of AES-128
            W0,W1,....W4R-1 are the 32-bit words of the expanded key

            Then for i = 0...4R-1:
                Wi = Ki    if i < N
                Wi = W[i-N] ^ SubWord(RotWord(Wi-1)) ^ rcon[i/N]  if i >= N and i mod N == 0
                Wi = W[i-N] ^ W[i-1]   otherwise
    */

   // first round key is initial key
   for(i = 0 ; i < NC ; i++){

    RoundKey[(i * 4) + 0] = Key[(i * 4) + 0];
    RoundKey[(i * 4) + 1] = Key[(i * 4) + 1];
    RoundKey[(i * 4) + 2] = Key[(i * 4) + 2];
    RoundKey[(i * 4) + 3] = Key[(i * 4) + 3];

   }

   // other keys are found from pervious round keys
   for(i = NC ; i < NC * (NR + 1) ; i++){

    {

        k = (i - 1) * 4;
        tempa[0] = RoundKey[k + 0];
        tempa[1] = RoundKey[k + 1];
        tempa[2] = RoundKey[k + 2];
        tempa[3] = RoundKey[k + 3];
    }

    if((i % NC) == 0){

        // Rotate the word
        RotateWord(tempa);

        // Get the rotated word equavlent value from the S Network
        GetSubWord(tempa);

        tempa[0] = tempa[0] ^ Rcon[i/NC];
    }

    j = i * 4;
    k = (i - NC) * 4;
    RoundKey[j + 0] = RoundKey[k + 0] ^ tempa[0];
    RoundKey[j + 1] = RoundKey[k + 1] ^ tempa[1];
    RoundKey[j + 2] = RoundKey[k + 2] ^ tempa[2];
    RoundKey[j + 3] = RoundKey[k + 3] ^ tempa[3];

   }

}

void AES_Init(struct AES_ctx* ctx, uint8_t * key){

    KeyExpansion(ctx->RoundKey , key);
}


static void AddRoundKey(uint8_t round , state_t * state , const uint8_t * Roundkey){

    uint8_t i , j;
    for(i = 0 ; i < 4 ; i++){

        for(j = 0 ; j < 4 ; j++){

            (*state)[i][j] ^= Roundkey[(round * NC * 4) + (i * NC) + j];
        }
    }
}

static void SubBytes(state_t * state){

    uint8_t i , j;
    for(i = 0 ; i < 4 ; i++){

        for(j = 0 ; j < 4 ; j++){

            (*state)[i][j] ^= getSBoxValue((*state)[i][j]);
        }
    }
}

static void ShiftRows(state_t * state){
    uint8_t temp;


    // Rotate second Row to the left by 1
    temp = (*state)[1][0];
    (*state)[1][0] = (*state)[1][1];
    (*state)[1][1] = (*state)[1][2];
    (*state)[1][2] = (*state)[1][3];
    (*state)[1][3] = temp;

    // Rotate third Row to the left by 2
    temp = (*state)[2][0];
    (*state)[2][0] = (*state)[2][2];
    (*state)[2][2] = temp;

    temp = (*state)[2][1];
    (*state)[2][1] = (*state)[2][3];
    (*state)[2][3] = temp;

    // Rotate fourth Row to the left by 3
    temp = (*state)[3][0];
    (*state)[3][0] = (*state)[3][3];
    (*state)[3][3] = (*state)[3][2];
    (*state)[3][2] = (*state)[3][1];
    (*state)[3][1] = temp;

}


static uint8_t gm2(uint8_t x){

    // the number is multplyed by 2
    // If the most significant bit of the number is 1
    // then output will be xored with 0x1b to prevent overflow
    return ((x << 1) ^ (((x >> 7) & 1) * 0x1b));

}

static uint8_t gm3(uint8_t x){

    // the number is multplyed by 2
    // If the most significant bit of the number is 1
    // then output will be xored with 0x1b to prevent overflow
    return gm2(x) ^ x;

}

static void mixword(uint8_t * word){

    uint8_t b0 , b1 , b2 , b3;

    b0 = word[0];
    b1 = word[1];
    b2 = word[2];
    b3 = word[3];

    word[0] = gm2(b0) ^ gm3(b1) ^ b2 ^ b3;
    word[1] = b0 ^ gm2(b1) ^ gm3(b2) ^ b3;
    word[2] = b0 ^ b1 ^ gm2(b2) ^ gm3(b3);
    word[0] = gm3(b0) ^ b1 ^ b2 ^ gm2(b3);
}


static void MixColumns(state_t * state){

    uint8_t i;

    for(i = 0 ; i < 4 ; i++){

        mixword((uint8_t *)(state)[i]);
    }
}

static void Cipher(state_t * state , const uint8_t * roundkey){

    uint8_t round = 0;

    // Add the first round key before starting
    AddRoundKey(0 , state , roundkey);

    // There will be NR - 1 identical rounds
    for(round = 1 ;  ; round++){
        SubBytes(state);
        ShiftRows(state);

        if(round == NR){

            break;
        }

        MixColumns(state);
        AddRoundKey(round , state , roundkey);
    }

    // Last round
    AddRoundKey(NR , state , roundkey);
}

void AES_Encrypt(const struct AES_ctx * ctx , uint8_t * buffer ){

    Cipher((state_t *)buffer , ctx->RoundKey);
}

static void RotateWord(uint8_t * word){

    // This function do one-byte left circular shift
    // RotWord([b0 b1 b2 b3]) = [b1 b2 b3 b0]

    uint8_t temp = word[0];
    word[0] = word[1];
    word[1] = word[2];
    word[2] = word[3];
    word[3] = temp;
}

static void GetSubWord(uint8_t * word){

    word[0] = getSBoxValue(word[0]);
    word[1] = getSBoxValue(word[1]);
    word[2] = getSBoxValue(word[2]);
    word[3] = getSBoxValue(word[3]);
}
