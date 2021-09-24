#include <krnl.h>
// one task loops and blink
// k_wait ensures proper timing
// how does t2 runs ??? and why

struct k_t *p1, *p2;


// the high priority task
void t1()
{
  int v;

  while (1) {

    k_eat_ticks(2000);  // eat time for 1000 ticks so mimic cpu usage == 2000 msec
    k_sleep(1000);
  }
}

void t2()
{
  int i = 0;
  while (1) {
    Serial.println(i++);

    if (10000 <= k_millis()) {
      while (1) ;  // hang here
    }

  }
  // do we keep up in pace ?
}


void setup()
{
  int res;
  Serial.begin(115200);
  while (! Serial) ;
  pinMode(13, OUTPUT);

  k_init(2, 0, 0); // init with space for one task
  //         |--- no of mg Queues (0)
  //       |----- no of semaphores (0)
  //     |------- no of tasks (2)

  // priority low number higher priority than higher number
  p1 = k_crt_task(t1, 10, 100); // t1 as task, priority 9, 100 B stak
  p2 = k_crt_task(t2, 10, 100); // t2 as task, priority 10, 100 B stak

  Serial.println("bef start");
  res = k_start(1); // 1 milli sec tick speed
  // you will never return from k_start
  Serial.print("ups an error occured: "); Serial.println(res);
  while (1) ;
}

void loop() {}
