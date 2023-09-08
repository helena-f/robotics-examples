#include <stdlib.h>
#include <stdio.h>

// 12.10 (Reversing the Words of a Sentence)
// Write a program that inputs a line of text and uses a stack to print the line reversed.
struct stackNode
{
    char data;
    struct stackNode *next;
};

void printStack(struct stackNode *currentPtr);
char pop(struct stackNode** topPtr);
void push(struct stackNode** topPtr, int value);

int main()
{
    printf("Enter text: ");
    
    struct stackNode* head = NULL;
    char c;
    while ( c!= '\n' )
    {
        scanf("%c", &c);
        push(&head, c);
    }
    
    printStack(head);

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
            printf(" %c ", currentPtr->data);
            currentPtr = currentPtr->next;
        }
    }
}