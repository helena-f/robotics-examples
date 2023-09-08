#include <stdio.h>
#include <stdlib.h>

struct Node {
    char data;
    struct Node *next;
};

void printList(struct Node *n);
void concatenate(struct Node *a, struct Node *b);

int main() {
    struct Node *head = NULL;
    struct Node *second = NULL;
    struct Node *third = NULL;
    struct Node *fourth = NULL;
    struct Node *fifth = NULL;
    struct Node *sixth = NULL;

    head = (struct Node *)malloc(sizeof(struct Node));
    second = (struct Node *)malloc(sizeof(struct Node));
    third = (struct Node *)malloc(sizeof(struct Node));

    head->data = 'A';
    head->next = second;

    second->data = 'B';
    second->next = third;

    third->data = 'C';
    third->next = NULL;

    printList(head);

    fourth = (struct Node *)malloc(sizeof(struct Node));
    fifth = (struct Node *)malloc(sizeof(struct Node));
    sixth = (struct Node *)malloc(sizeof(struct Node));

    fourth->data = 'D';
    fourth->next = fifth;

    fifth->data = 'E';
    fifth->next = sixth;

    sixth->data = 'F';
    sixth->next = NULL;

    printList(fourth);

    printf("\n");

    concatenate(head, fourth);
    printList(head);
    return 0;
}

void printList(struct Node *n){
    while (n != NULL){
        printf(" %c ", n->data);
        n = n->next;
    }
}

void concatenate(struct Node *a, struct Node *b) {
    if (a->next == NULL) {
        a->next = b;
    } else {
        concatenate(a->next, b);
    }
}

#include <stdio.h>
#include <stdlib.h>

struct Node {
    int data;
    struct Node *next;
};

void printList(struct Node *n);
void addNode(struct Node *head, int value);
struct Node *sortedMerge(struct Node *a, struct Node *b);

int main() {
    // struct Node* myList = (struct Node*) malloc(sizeof(struct Node));
    // myList->data = 0;
    // myList->next = NULL;

    struct Node *head = NULL;
    struct Node *second = NULL;
    struct Node *third = NULL;
    struct Node *fourth = NULL;
    struct Node *fifth = NULL;
    struct Node *sixth = NULL;
    struct Node *seventh = NULL;
    struct Node *eigth = NULL;
    struct Node *ninth = NULL;
    struct Node *tenth = NULL;

    head = (struct Node *)malloc(sizeof(struct Node));
    second = (struct Node *)malloc(sizeof(struct Node));
    third = (struct Node *)malloc(sizeof(struct Node));
    fourth = (struct Node *)malloc(sizeof(struct Node));
    fifth = (struct Node *)malloc(sizeof(struct Node));
    sixth = (struct Node *)malloc(sizeof(struct Node));
    seventh = (struct Node *)malloc(sizeof(struct Node));
    eigth = (struct Node *)malloc(sizeof(struct Node));
    ninth = (struct Node *)malloc(sizeof(struct Node));
    tenth = (struct Node *)malloc(sizeof(struct Node));

    head->data = 1;
    head->next = third;
    third->data = 3;
    third->next = fifth;
    fifth->data = 5;
    fifth->next = seventh;
    seventh->data = 7;
    seventh->next = ninth;
    ninth->data = 9;
    ninth->next = NULL;

    second->data = 2;
    second->next = fourth;
    fourth->data = 4;
    fourth->next = sixth;
    sixth->data = 6;
    sixth->next = eigth;
    eigth->data = 8;
    eigth->next = tenth;
    tenth->data = 10;
    tenth->next = NULL;

    sortedMerge(head, second);

    printList(head);
}

struct Node *sortedMerge(struct Node *a, struct Node *b)
{
    struct Node *result = NULL;

    if (a == NULL)
    {
        return (b);
    }
    else if (b == NULL)
    {
        return (a);
    }

    if (a->data <= b->data)
    {
        result = a;
        result->next = sortedMerge(a->next, b);
    }
    else
    {
        result = b;
        result->next = sortedMerge(a, b->next);
    }
    return (result);
}

void printList(struct Node *n)
{
    while (n != NULL)
    {
        printf(" %d ", n->data);
        n = n->next;
    }
}

void addNode(struct Node *head, int value)
{
    struct Node *newNode = (struct Node *)malloc(sizeof(struct Node));

    if (newNode != NULL)
    {
    }
    while (head != NULL)
    {
        head = head->next;
    }
    newNode->data = 1;
    head->next = newNode;
}

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

struct Node {
    int data;
    struct Node* next; 
}; 

void addNode(struct Node *head, int value);
void printList (struct Node* list);
void append (struct Node** headRef, int value);
int sumList ( struct Node* head) ;
float average (struct Node* head, int sum) ;

int main () 
{
    srand(time(NULL));

    struct Node* head = NULL;

    int n = 0;
    for (int i = 0; i < 25; i++) {
        n = rand() %101;
        append(&head, n);
    }
    printList(head);

    printf(" \n sum = %d", sumList(head));
    printf(" \n average = %f", average(head, sumList(head)));
}
void append (struct Node** headRef, int value)
{
    struct Node* newNode = (struct Node *)malloc(sizeof(struct Node)); // create node
    newNode->data = value; // assign value to new node
    newNode->next = (*headRef); // after new node is pointer to head
    (*headRef) = newNode; // original pointer equals new node
}

float average (struct Node* head, int sum) 
{
    float avg = 0.0;
    int counter = 0;
    while ( head != NULL)
    {
        counter++;
        head = head-> next;
    }

    avg = sum/counter;

    return avg;
}

int sumList ( struct Node* head) 
{
    int sum = 0;
    while (head != NULL) 
    {
        sum += head->data;
        head = head->next;
    }
    return sum;
}

void printList (struct Node* list)
{
    while (list != NULL ) {
        printf(" %d ", list->data);
        list = list->next; 
    }
}