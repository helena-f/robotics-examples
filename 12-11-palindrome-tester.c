#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>

//12.11 (Palindrome Tester) Write a program that uses a stack to determine whether a string is a palindrome 
//(i.e., the string is spelled identically backward and forward). The program should ignore spaces and punctuation.

struct stackNode {
    char data;
    struct stackNode* next;
};

bool isPalindrome(char *text, struct stackNode** n);
void stripString( char *str);
void push(struct stackNode** topPtr, int value);
void printStack(struct stackNode* currentPtr);
char pop(struct stackNode** topPtr);

int main () 
{
    struct stackNode* head = NULL;
    struct stackNode* second = NULL;

    printf("Enter text: ");

    char text [10];
    scanf("%s", text);

    stripString(text);
    for (int i = 0; text[i] != '\0'; i++) {
        push(&head, text[i]);
    }

    if (isPalindrome(text, &head)) {
        printf("\n%s is a palindrome.", text);
    } else {
        printf("\n%s is not a palindrome.", text);
    }
}

bool isPalindrome (char *text, struct stackNode** n) 
{
    while (*text) 
    {
        if (*text == pop(n)){
            text++;
        } else {
            return false;
        }
    }
    return true;
}

void stripString (char *str)
{
    char *dest = str;
    while (*str)
    {
        if (ispunct((unsigned char)*str) || *str == ' '){
            str++;
        } else if (str == dest) {
            str++;
            dest++;
        } else {
            *dest++ = *str++;
        }
    }
    *dest = 0;
}

// push a value in the stack
void push(struct stackNode** topPtr, int value)
{
    struct stackNode* newPtr = (struct stackNode *)malloc(sizeof(struct stackNode *));
    if (newPtr != NULL ) 
    {
        newPtr->data = value;
        newPtr->next = *topPtr;
        *topPtr = newPtr; 
    } else {
        printf("No memory left.");
    }
}

// pop out a value in the stack
char pop(struct stackNode** topPtr)
{
    struct stackNode* temp = *topPtr;
    char popValue = (*topPtr)->data;
    (*topPtr) = (*topPtr)->next;
    free(temp);
    return popValue;
}

void printStack(struct stackNode* currentPtr)
{
    if (currentPtr == NULL)
    {
        puts("The stack is empty. \n");
    }
    else
    {
        while (currentPtr != NULL)
        {
            printf("%c", currentPtr->data);
            currentPtr = currentPtr->next;
        }
    }
}
