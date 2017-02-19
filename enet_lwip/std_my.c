#include "std_my.h"

int my_STR_SUBn( char *a , char *b,  int n )
{
    while( n ) {
        if( *a != *b )
            return 0;
        a++;
        b++;
        n--;
    }
    return 0xffff;
}
