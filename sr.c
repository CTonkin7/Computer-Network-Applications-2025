#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12     /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) {
    checksum += (int)(packet.payload[i]);
  }
  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[SEQSPACE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

/* Sender ACK and Timer status initialisation */
static int acked[SEQSPACE];
static int timer_status[SEQSPACE];

/* Receiver buffer and received initialisations */
static struct pkt recv_buffer[SEQSPACE];
static int received[SEQSPACE];

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) {
      sendpkt.payload[i] = message.data[i];
    }
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    windowlast = (windowlast + 1) % SEQSPACE; 
    buffer[A_nextseqnum] = sendpkt;
    windowcount++;

    acked[A_nextseqnum] = 0; /* track current ACK */
    timer_status[A_nextseqnum] = 1; /* create status of timer */

    /* send out packet */
    if (TRACE > 0){
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    }
    tolayer3 (A, sendpkt);
    starttimer(A,RTT + A_nextseqnum); /* begin timer for packet. */

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
In this practical this will always be an ACK as B never sends data.
*/


void A_input(struct pkt packet)
{
  int acknum = packet.acknum; /* initialise acknumber variable to current packet*/
  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0) {
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    }
    
    if (!acked[acknum]) {
      acked[acknum] = 1; /* mark received ACK*/
      new_ACKs++;
      stoptimer(A);
      timer_status[acknum] = 0; /* stop timer after ACK received*/
      
      if (TRACE > 0) {
      printf("----A: ACK %d is not a duplicate\n", acknum);
      }
      /* slide window if base is acked */
      while (acked[windowfirst] && windowcount > 0) {
        windowfirst = (windowfirst + 1) % SEQSPACE;
        windowcount--;
      }

    } else {
      if (TRACE > 0)
        printf("----A: duplicate ACK received for packet %d\n", acknum);
    } 
  }
  else { 
      if (TRACE > 0)
        printf ("----A: corrupted ACK is received, do nothing!\n");
    }
}


/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;

  if (TRACE > 0){
    printf("----A: Timer Expired, check for packet to resend\n");
  }

  for(i=0; i<SEQSPACE; i++) {
    
    if (timer_status[i] && !acked[i]){
      if (TRACE > 0){
        printf("----A: Resending packet %d\n", i);
      }
      tolayer3(A,buffer[i]);
      starttimer(A,RTT + i); /* restart timer */
      packets_resent++;
      /*break; send only one packet on timeout */
    }
  }
}       



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  windowcount = 0;

  /* Initialise Selective Repeat Tracking arrays with ACK status and timer status*/
  for (i = 0; i < SEQSPACE; i++) {
    received[i] = 0;
    acked[i] = 0; /* not ACKed yet */
    timer_status[i] = 0; /* timer not running yet */
  }

}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt ackpkt;
  int i;

  /* check for corruption: */
  if (!IsCorrupted(packet)){
    int seq = packet.seqnum;

    if (!received[seq]){
      received[seq] = 1;
      recv_buffer[seq] = packet;

      packets_received++;

      if (TRACE > 0) {
        printf("----B: Packet %d is correctly received, send ACK!\n", seq);
      }
    } else {
      if (TRACE > 0) {
        printf("----B: Duplicate packet %d received, resend ACK\n", seq);
      }
    }
    
    ackpkt.seqnum = B_nextseqnum;
    ackpkt.acknum = seq;

    for (i = 0; i < 20; i++) {
      ackpkt.payload[i] = 0;
    }
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B,ackpkt);
    B_nextseqnum = (B_nextseqnum + 1) % 2;

    while (received[expectedseqnum % SEQSPACE]) {
      tolayer5(B, recv_buffer[expectedseqnum % SEQSPACE].payload);
      received[expectedseqnum % SEQSPACE] = 0;
      expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
    }
  } else {
    if (TRACE > 0){
      printf("----B: Corrupted packet received, no ACK sent\n");
    }
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

