#include <string.h>
#include <stddef.h>
#include "tkjhat/sdk.h"
#include "translatemorse.h"

// Define a struct for a lookup table
typedef struct {
    const char * morse;
    char letter;
} MorseLookup;

static const MorseLookup morsetable[] = {
    {".-", 'A'},   {"-...", 'B'}, {"-.-.", 'C'}, {"-..", 'D'},
    {".", 'E'},    {"..-.", 'F'}, {"--.", 'G'},  {"....", 'H'},
    {"..", 'I'},   {".---", 'J'}, {"-.-", 'K'},  {".-..", 'L'},
    {"--", 'M'},   {"-.", 'N'},   {"---", 'O'},  {".--.", 'P'},
    {"--.-", 'Q'}, {".-.", 'R'},  {"...", 'S'},  {"-", 'T'},
    {"..-", 'U'},  {"...-", 'V'}, {".--", 'W'},  {"-..-", 'X'},
    {"-.--", 'Y'}, {"--..", 'Z'}, {"", ' '}
};

// Size of the table we are going to iterate through
#define MORSE_TABLE_SIZE (sizeof(morsetable) / sizeof(morsetable[0]))

// display_morse takes a pointer to morse string and the string size as arguments
// Decodes the string as latin alphabet and writes it to OLED
void display_morse(char * line, int size) {
    clear_display(); // Clears previous message from the screen
    char buf[20] = ""; // String buffer for writing text to OLED
    size_t pos = 0; // Current index of buf
    char temp[size]; // Local copy of line
    strcpy(temp, line); // Copy contents of line to temp (including the null terminator of course)
    char *ptr = temp; // Pointer to temp (for the use of strsep())
    char *tok; // Token extracted from strsep()

    // Loop that uses strsep() to tokenize the morse string with space as the delimiter
    // Then we loop through the lookuptable and check which letter the current token matches
    // Spaces between words (aka two spaces in a row) are interpreted as "" by strsep() 
    while(((tok = strsep(&ptr, " ")) != NULL) && (pos < (sizeof(buf) - 1))) {
        for (size_t i = 0; i < MORSE_TABLE_SIZE; i++) {
            if (strcmp(tok, morsetable[i].morse) == 0) {
                buf[pos++] = morsetable[i].letter;
                break;
            }
        }
    }
    buf[pos] = '\0'; // Loops until last index of buf is achieved and terminates the latin letter string
    write_text(buf); // Writes to OLED
}