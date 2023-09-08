//Write a recursive function outputTree to display a binary tree on the screen, sideways. 
//The function should output the tree row-by-row with the top of the tree at the left, 
//and the bottom of the tree towards the right. Each row is output vertically. 

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

struct treeNode {
    struct treeNode* leftPtr;
    int data;
    struct treeNode* rightPtr;
};

void outputTree(struct treeNode* treePtr, int totalSpaces);
void insertNode(struct treeNode** treePtr, int value);

int main () 
{
    srand(time(NULL));
    struct treeNode* head = NULL; 

    for (int i = 0; i < 10; i++)
    {
        int n = rand() % 15;
        insertNode(&head, n);
    }

    outputTree(head, 0);

}

void outputTree(struct treeNode* treePtr, int totalSpaces)
{
    if (treePtr)
    {
        outputTree(treePtr->rightPtr, totalSpaces + 5);
        printf("\n");
        for (int i = 1; i < totalSpaces; i++)
        {
            printf(" ");
        }
        printf("%d", treePtr->data);
        outputTree(treePtr->leftPtr, totalSpaces+5);
    }
    
}

// insert a node into the tree
void insertNode(struct treeNode** treePtr, int value)
{
    // if treePtr is NULL
    if (*treePtr == NULL)
    {
        // dynamically allocate memory
        *treePtr = malloc(sizeof(struct treeNode));

        // if memory was allocated, insert node
        if (*treePtr)
        {
            (*treePtr)->data = value;
            (*treePtr)->leftPtr = NULL;
            (*treePtr)->rightPtr = NULL;
        }
        else
        {
            printf("%d not inserted. No memory available.\n", value);
        }
    }
    else
    {
        // data to insert is less than data in current nodw
        if (value < (*treePtr)->data)
        {
            insertNode(&((*treePtr)->leftPtr), value);
        }
        else if (value > (*treePtr)->data)
        {
            insertNode(&((*treePtr)->rightPtr), value);
        }
    }
}
