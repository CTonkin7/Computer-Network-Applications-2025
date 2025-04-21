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
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
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
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

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

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

/* Sender ACK and Timer status initialisation */
static int acked[WINDOWSIZE];
static int timer_status[WINDOWSIZE];

/* Receiver buffer and received initialisations */
static struct pkt recv_buffer[WINDOWSIZE];
static int received[WINDOWSIZE];

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    windowlast = (windowlast + 1) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    windowcount++;

    acked[sendpkt.seqnum % WINDOWSIZE] = 0; /* track current ACK */
    timer_status[sendpkt.seqnum % WINDOWSIZE] = 1; /* create status of timer */

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    starttimer(A,RTT); /* begin timer for packet. */

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

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;

    int acknum = packet.acknum; /* initialise acknumber variable to current packet*/

    if (acked[acknum % WINDOWSIZE] == 0) {
      acked[acknum % WINDOWSIZE] = 1; /* mark received ACK*/
      timer_status[acknum % WINDOWSIZE] = 0; /* stop timer after ACK received*/
      
      if (TRACE > 0)
      printf("----A: Marked Packet %d as acknowledged\n", acknum);

      /* slide window if base is acked */
      while (acked[buffer[windowfirst].seqnum % WINDOWSIZE] && windowcount > 0) {
        windowfirst = (windowfirst + 1) % WINDOWSIZE;
        windowcount--;
      }

      stoptimer(A); /* End timer after acknowledgement */
      if(windowcount > 0) /* begin new timer */
        starttimer(A,RTT);
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
  if (TRACE > 0){
    printf("----A: Timer Expired, check for packet to resend\n");
  }

  for(i=0; i<windowcount; i++) {
    int idx = (windowfirst + i) % WINDOWSIZE;
    int seq = buffer[idx].seqnum;
    
    if ((acked[seq % WINDOWSIZE] == 0 )&& (timer_status[seq % WINDOWSIZE] == 1)){
      if (TRACE > 0){
        printf("----A: Resending packet %d\n", seq);
      }
      tolayer3(A,buffer[idx]);
      packets_resent++;
      starttimer(A,RTT); /* restart timer */
      break; /* send only one packet on timeout */
    }
  }
}       



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  windowcount = 0;

  /* Initialise Selective Repeat Tracking arrays with ACK status and timer status*/
  for (int i = 0; i < WINDOWSIZE; i++) {
    acked[i] = 0; // not ACKed yet
    timer_status[i] = 0; // timer not running yet
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

    if (received[seq % WINDOWSIZE] == 0){
      received[seq % WINDOWSIZE] = 1;
      recv_buffer[seq % WINDOWSIZE] = packet;

      if (TRACE > 0) {
        printf("----B: Packet %d received and buffered\n", seq);
      }
    } else {
      if (TRACE > 0) {
        printf("----B: Duplicate packet %d received, resend ACK\n", seq);
      }
    }
    
    ackpkt.seqnum = B_nextseqnum;
    ackpkt.acknum = seq;
    B_nextseqnum = (B_nextseqnum + 1) % 2;

    for (i = 0; i < 20; i++) {
      ackpkt.payload[i] = '0';
    }
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B,ackpkt);

    while (received[expectedseqnum % WINDOWSIZE]) {
      tolayer5(B, recv_buffer[expectedseqnum % WINDOWSIZE].payload);
      received[expectedseqnum % WINDOWSIZE] = 0;
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

