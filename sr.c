#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h> // Added for memcpy
#include "emulator.h"
#include "sr.h" // Changed from gbn.h

/* ******************************************************************
   Selective Repeat protocol.  Adapted from J.F.Kurose
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
   - added GBN implementation (Base for SR)
   - Implemented Selective Repeat receiver logic (Step 1)
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
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
// ... (A side code remains the same as gbn.c for now) ...

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

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

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    if (windowcount == 1)
      starttimer(A,RTT);

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
  int ackcount = 0;
  int i;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;

    /* check if new ACK or duplicate */
    if (windowcount != 0) {
          int seqfirst = buffer[windowfirst].seqnum;
          int seqlast = buffer[windowlast].seqnum;
          /* check case when seqnum has and hasn't wrapped */
          if (((seqfirst <= seqlast) && (packet.acknum >= seqfirst && packet.acknum <= seqlast)) ||
              ((seqfirst > seqlast) && (packet.acknum >= seqfirst || packet.acknum <= seqlast))) {

            /* packet is a new ACK */
            if (TRACE > 0)
              printf("----A: ACK %d is not a duplicate\n",packet.acknum);
            new_ACKs++;

            /* cumulative acknowledgement - determine how many packets are ACKed */
            if (packet.acknum >= seqfirst)
              ackcount = packet.acknum + 1 - seqfirst;
            else
              ackcount = SEQSPACE - seqfirst + packet.acknum;

	    /* slide window by the number of packets ACKed */
            windowfirst = (windowfirst + ackcount) % WINDOWSIZE;

            /* delete the acked packets from window buffer */
            for (i=0; i<ackcount; i++)
              windowcount--;

	    /* start timer again if there are still more unacked packets in window */
            stoptimer(A);
            if (windowcount > 0)
              starttimer(A, RTT);

          }
        }
        else
          if (TRACE > 0)
        printf ("----A: duplicate ACK received, do nothing!\n");
  }
  else
    if (TRACE > 0)
      printf ("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  for(i=0; i<windowcount; i++) {

    if (TRACE > 0)
      printf ("---A: resending packet %d\n", (buffer[(windowfirst+i) % WINDOWSIZE]).seqnum);

    tolayer3(A,buffer[(windowfirst+i) % WINDOWSIZE]);
    packets_resent++;
    if (i==0) starttimer(A,RTT); // GBN retransmits and restarts timer for the first packet
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
}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B (used for ACK seqnum) */

// SR Receiver specific variables
static struct pkt rcv_buffer[WINDOWSIZE]; // Buffer for storing received packets
static bool buffered[SEQSPACE];           // To track which sequence numbers are buffered

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int packet_seqnum = packet.seqnum;

  /* if not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----B: uncorrupted packet %d received\n", packet_seqnum);
    packets_received++;

    // Check if the received packet is within the receiver's window [expectedseqnum, expectedseqnum + WINDOWSIZE - 1] (modulo SEQSPACE)
    bool in_window = false;
    int end_window = (expectedseqnum + WINDOWSIZE - 1) % SEQSPACE;

    if (expectedseqnum <= end_window) { // Window does not wrap around sequence space
        if (packet_seqnum >= expectedseqnum && packet_seqnum <= end_window) {
            in_window = true;
        }
    } else { // Window wraps around sequence space
        if (packet_seqnum >= expectedseqnum || packet_seqnum <= end_window) {
            in_window = true;
        }
    }


    if (in_window) {
        if (TRACE > 0)
            printf("----B: packet %d is within the receiver window\n", packet_seqnum);

        // Send individual ACK for the received packet
        sendpkt.seqnum = B_nextseqnum; // You can still use a simple sequence number for ACKs if not implementing bidirectional
        sendpkt.acknum = packet_seqnum;
        // we don't have any data to send back.  fill payload with 0's
        for ( i=0; i<20 ; i++ )
            sendpkt.payload[i] = '0';
        sendpkt.checksum = ComputeChecksum(sendpkt);
        tolayer3(B, sendpkt);
        if (TRACE > 0)
            printf("----B: sending ACK for packet %d\n", packet_seqnum);


        // Calculate buffer index
        int buffer_index = (packet_seqnum - expectedseqnum + SEQSPACE) % SEQSPACE;

        // Buffer the packet if it's a new packet within the window
        if (!buffered[buffer_index]) {
            rcv_buffer[buffer_index] = packet; // Copy the packet
            buffered[buffer_index] = true;
            if (TRACE > 0)
                 printf("----B: buffering packet %d at index %d\n", packet_seqnum, buffer_index);
        } else {
             if (TRACE > 0)
                 printf("----B: packet %d is a duplicate within the window, already buffered\n", packet_seqnum);
        }


        // Check if packets can be delivered to Layer 5
        while (buffered[0]) { // While the packet at the start of the window is buffered
            if (TRACE > 0)
                printf("----B: delivering packet %d to layer 5\n", expectedseqnum);
            tolayer5(B, rcv_buffer[0].payload);

            // Slide the receiver window
            expectedseqnum = (expectedseqnum + 1) % SEQSPACE;

            // Shift buffered packets and flags
            for (i = 0; i < WINDOWSIZE - 1; i++) {
                rcv_buffer[i] = rcv_buffer[i+1];
                buffered[i] = buffered[i+1];
            }
            // Mark the last position as not buffered
            buffered[WINDOWSIZE - 1] = false;

             if (TRACE > 0)
                 printf("----B: receiver window slides to expected seq %d\n", expectedseqnum);
        }

    } else {
      if (TRACE > 0)
        printf("----B: packet %d is outside the receiver window [%d, %d], discarding\n", packet_seqnum, expectedseqnum, end_window);
      // If packet is outside the window, it's either a packet that has already been ACKed and delivered,
      // or a packet far ahead that we are not ready for. In SR, we typically just discard.
      // However, sending an ACK for a duplicate (already delivered) packet is also a valid strategy
      // to help the sender. Let's add sending ACK for already delivered packets if not corrupted.
       if (packet_seqnum < expectedseqnum) { // This packet has likely already been delivered
           if (TRACE > 0)
              printf("----B: packet %d is a duplicate (already delivered), resending ACK\n", packet_seqnum);
           sendpkt.seqnum = B_nextseqnum;
           sendpkt.acknum = packet_seqnum;
           for ( i=0; i<20 ; i++ )
              sendpkt.payload[i] = '0';
           sendpkt.checksum = ComputeChecksum(sendpkt);
           tolayer3(B, sendpkt);
            if (TRACE > 0)
                printf("----B: resending ACK for packet %d\n", packet_seqnum);
       }
    }
  }
  else {
    if (TRACE > 0)
      printf ("----B: corrupted packet received, discarding\n");
    // In SR, if a packet is corrupted, the receiver discards it.
    // The sender's timer for that packet will eventually time out, triggering retransmission.
    // No ACK is sent for corrupted packets.
  }

  // This simple ACK sequence number increment is fine for unidirectional data transfer from A to B
  B_nextseqnum = (B_nextseqnum + 1) % 2;
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1; // Initial sequence number for ACKs from B (can be anything, just needs to change for checksum)

  // Initialize SR receiver buffer and flags
  for (int i = 0; i < WINDOWSIZE; i++) {
      buffered[i] = false;
  }
   if (TRACE > 0)
       printf("----B: Initialized receiver buffer and flags.\n");
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