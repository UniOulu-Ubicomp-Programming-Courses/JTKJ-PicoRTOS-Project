#include <string.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include "tkjhat/sdk.h"
#include "translatemorse.h"
 
void display_morse(char * line, int size) {
        clear_display();
        char buf[20];
        int pos = 0;
        char temp[size];
        strcpy(temp, line);
        char *ptr = temp;
        char *tok;
        while(((tok = strsep(&ptr, " ")) != NULL) && (pos < (sizeof(buf) - 1))) {
            if (strcmp(tok, ".-") == 0) {
                buf[pos++] = 'A';
            }
            else if (strcmp(tok, "-...") == 0) {
                buf[pos++] = 'B';
            }
            else if (strcmp(tok, "-.-.") == 0) {
                buf[pos++] = 'C';
        }
            else if (strcmp(tok, "-..") == 0) {
                buf[pos++] = 'D';
        }
            else if (strcmp(tok, ".") == 0) {
                buf[pos++] = 'E';
        }
            else if (strcmp(tok, "..-.") == 0) {
                buf[pos++] = 'F';
        }
            else if (strcmp(tok, "--.") == 0) {
                buf[pos++] = 'G';
        }
            else if (strcmp(tok, "....") == 0) {
                buf[pos++] = 'H';
        }
            else if (strcmp(tok, "..") == 0) {
                buf[pos++] = 'I';
        }
            else if (strcmp(tok, ".---") == 0) {
                buf[pos++] = 'J';
        }
            else if (strcmp(tok, "-.-") == 0) {
                buf[pos++] = 'K';
        }
            else if (strcmp(tok, ".-..") == 0) {
                buf[pos++] = 'L';
        }
            else if (strcmp(tok, "--") == 0) {
                buf[pos++] = 'M';
        }
            else if (strcmp(tok, "-.") == 0) {
                buf[pos++] = 'N';
        }
            else if (strcmp(tok, "---") == 0) {
                buf[pos++] = 'O';
        }
            else if (strcmp(tok, ".--.") == 0) {
                buf[pos++] = 'P';
        }
            else if (strcmp(tok, "--.-") == 0) {
                buf[pos++] = 'Q';
        }
            else if (strcmp(tok, ".-.") == 0) {
                buf[pos++] = 'R';
        }
            else if (strcmp(tok, "...") == 0) {
                buf[pos++] = 'S';
        }
            else if (strcmp(tok, "-") == 0) {
                buf[pos++] = 'T';
        }
            else if (strcmp(tok, "..-") == 0) {
                buf[pos++] = 'U';
        }
            else if (strcmp(tok, "...-") == 0) {
                buf[pos++] = 'V';
        }
            else if (strcmp(tok, ".--") == 0) {
                buf[pos++] = 'W';
        }
            else if (strcmp(tok, "-..-") == 0) {
                buf[pos++] = 'X';
        }
            else if (strcmp(tok, "-.--") == 0) {
                buf[pos++] = 'Y';
        }
            else if (strcmp(tok, "--..") == 0) {
                buf[pos++] = 'Z';
        }
            else if (strcmp(tok, "") == 0) {
                buf[pos++] = ' ';
        }
    }
        buf[pos] = '\0';
        write_text(buf);
}