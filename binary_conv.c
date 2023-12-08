#include <stdio.h>

int main() {
    signed char binaryInt;

    while (fread(&binaryInt, sizeof(signed char), 1, stdin) == 1) {
        printf("%d\n", binaryInt);
    }

    return 0;
}

