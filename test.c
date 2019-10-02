#include <stdio.h>
#include <string.h>

typedef unsigned char BYTE;

//function to convert string to byte array
void string2ByteArray(char* input, BYTE* output)
{
    int loop;
    int i;
    
    loop = 0;
    i = 0;
    
    while(input[loop] != '\0')
    {
        output[i++] = input[loop++];
    }
}

int main(){
    char ascii_str[] = "Hello world!";
    int len = strlen(ascii_str);
    BYTE arr[len];
    int i;
    
    //converting string to BYTE[]
    string2ByteArray(ascii_str, arr);
    
    //printing
    printf("ascii_str: %s\n", ascii_str);
    printf("byte array is...\n");
    for(i=0; i<len; i++)
    {
        printf("%c - %X\n", ascii_str[i], arr[i]);
    }
    printf("\n");
    
    return 0;
}