/* =====================================================================
 *  robot_countGride.ino      ภารกิจ Robot (ใบงาน e33)
 * =====================================================================
 *
 *  สนาม (o = วัตถุ, x = ตำแหน่งวางวัตถุ, [R]> = จุดเริ่มต้น Robot หันไปทางขวา)
 *
 *          o1                                  o3
 *     ─────┬───────┬───────┬───────┬─────      <- TOP
 *          │       │       │       │
 *     ──[R]>───────┼───────┼───────┼─────      <- MID
 *          │       │       │       │
 *     ─────┴───────┴───────┴───────┴─────      <- BOT
 *          o2      x3      x2      x1
 *          C1      C2      C3      C4
 *
 *  วัตถุ                    ->  ตำแหน่งวางที่ต้องไป
 *    o1 = บนสุดของ C1        ->  x1 = ล่างสุดของ C4
 *    o2 = ล่างสุดของ C1      ->  x2 = ล่างสุดของ C3
 *    o3 = บนสุดของ C4        ->  x3 = ล่างสุดของ C2
 *
 *  โจทย์ให้เริ่มหยิบวัตถุหมายเลขใดก่อนก็ได้ โปรแกรมนี้เลือกลำดับ  3 -> 1 -> 2
 *  เพราะหุ่นเริ่มต้นหันหน้าไปทางขวา จึงวิ่งไปเก็บ o3 ที่อยู่ทางขวาสุดก่อน
 *
 * ---------------------------------------------------------------------
 *  วิธีนับ numGride  (เหมือนในสไลด์ robot08 / robot09 / robot10)
 *  - countGrid() บวก 1 ทุกครั้งที่เจอ "เส้นตัด"
 *  - จุดที่หุ่นยืนอยู่ตอนเริ่ม (C1,MID) ไม่ถูกนับ  เส้นตัดแรกที่เจอคือ 1
 *  - ทุก case ที่มีคำสั่งจะ numGride++ เพิ่มอีก 1  เลข case จึงกระโดด
 *
 *  ตารางเส้นทาง
 *  n    จุด        ทิศ ก่อน->หลัง   คำสั่ง
 *  ---  ---------  --------------  ------------------------------------
 *   1   (C2,MID)   E               followLine
 *   2   (C3,MID)   E               followLine
 *   3   (C4,MID)   E -> N          turn90("LEFT")
 *   5   (C4,TOP)   N -> S          keep_item   หยิบวัตถุ 3
 *   7   (C4,MID)   S -> W          turn90("RIGHT")
 *   9   (C3,MID)   W               followLine
 *  10   (C2,MID)   W -> S          turn90("LEFT")
 *  12   (C2,BOT)   S -> N          place_item  วางวัตถุ 3 ที่ x3
 *  14   (C2,MID)   N -> W          turn90("LEFT")
 *  16   (C1,MID)   W -> N          turn90("RIGHT")
 *  18   (C1,TOP)   N -> S          keep_item   หยิบวัตถุ 1
 *  20   (C1,MID)   S -> E          turn90("LEFT")
 *  22   (C2,MID)   E               followLine
 *  23   (C3,MID)   E               followLine
 *  24   (C4,MID)   E -> S          turn90("RIGHT")
 *  26   (C4,BOT)   S -> N          place_item  วางวัตถุ 1 ที่ x1
 *  28   (C4,MID)   N -> W          turn90("LEFT")
 *  30   (C3,MID)   W               followLine
 *  31   (C2,MID)   W               followLine
 *  32   (C1,MID)   W -> S          turn90("LEFT")
 *  34   (C1,BOT)   S -> N          keep_item   หยิบวัตถุ 2
 *  36   (C1,MID)   N -> E          turn90("RIGHT")
 *  38   (C2,MID)   E               followLine
 *  39   (C3,MID)   E -> S          turn90("RIGHT")
 *  41   (C3,BOT)   S               place_item("STOP")  วางวัตถุ 2 ที่ x2
 *  42   จบภารกิจ -> stopRobot()
 * ===================================================================== */

#include "controlLibrary.h"

/* ---- ค่าที่ปรับจูนหน้างานได้ ------------------------------------------
 * ถ้าหุ่นเลยจุดตัดมากไป/น้อยไปก่อนเลี้ยว หรือหนีบวัตถุไม่ติด ให้แก้ 3 ค่านี้
 */
#define NUDGE_TURN    50   // ms เดินหน้าเลยจุดตัดก่อนจะเลี้ยว
#define NUDGE_PICK    50   // ms เดินหน้าเข้าหาวัตถุก่อนหนีบ
#define NUDGE_PLACE   30   // ms เดินหน้าก่อนวางวัตถุ

#define END_GRIDE     42   // นับถึงเลขนี้ = ภารกิจเสร็จ

int numGride = 0;

void turn90(String direction);
void keep_item(String direction);
void place_item(String direction);
void leaveGride();

void setup() {
  Serial.begin(9600);
  beginFnc();
}

void loop() {
  // ภารกิจจบแล้ว -> จอดค้างไว้ (กัน countGrid นับต่อแล้วหุ่นวิ่งออกนอกสนาม)
  if (numGride >= END_GRIDE) { stopRobot(); return; }

  numGride = countGrid(numGride);

  switch (numGride) {

    /* ---------- วัตถุ 3 : บนสุด C4  ->  วางที่ x3 : ล่างสุด C2 ---------- */
    case  3: turn90("LEFT");       numGride++; break;  // (C4,MID) หันขึ้นเหนือ
    case  5: keep_item("RIGHT");   numGride++; break;  // (C4,TOP) หยิบวัตถุ 3
    case  7: turn90("RIGHT");      numGride++; break;  // (C4,MID) หันไปทางซ้ายมือสนาม
    case 10: turn90("LEFT");       numGride++; break;  // (C2,MID) หันลงใต้
    case 12: place_item("RIGHT");  numGride++; break;  // (C2,BOT) วางวัตถุ 3

    /* ---------- วัตถุ 1 : บนสุด C1  ->  วางที่ x1 : ล่างสุด C4 ---------- */
    case 14: turn90("LEFT");       numGride++; break;  // (C2,MID) หันไปทางซ้ายมือสนาม
    case 16: turn90("RIGHT");      numGride++; break;  // (C1,MID) หันขึ้นเหนือ
    case 18: keep_item("RIGHT");   numGride++; break;  // (C1,TOP) หยิบวัตถุ 1
    case 20: turn90("LEFT");       numGride++; break;  // (C1,MID) หันไปทางขวามือสนาม
    case 24: turn90("RIGHT");      numGride++; break;  // (C4,MID) หันลงใต้
    case 26: place_item("RIGHT");  numGride++; break;  // (C4,BOT) วางวัตถุ 1

    /* ---------- วัตถุ 2 : ล่างสุด C1  ->  วางที่ x2 : ล่างสุด C3 -------- */
    case 28: turn90("LEFT");       numGride++; break;  // (C4,MID) หันไปทางซ้ายมือสนาม
    case 32: turn90("LEFT");       numGride++; break;  // (C1,MID) หันลงใต้
    case 34: keep_item("RIGHT");   numGride++; break;  // (C1,BOT) หยิบวัตถุ 2
    case 36: turn90("RIGHT");      numGride++; break;  // (C1,MID) หันไปทางขวามือสนาม
    case 39: turn90("RIGHT");      numGride++; break;  // (C3,MID) หันลงใต้
    case 41: place_item("STOP");   numGride++; break;  // (C3,BOT) วางวัตถุ 2 แล้วจบ

    default: followLine();
  }
}

/* =====================================================================
 *  ฟังก์ชันช่วย (จากสไลด์ robot10)
 * ===================================================================== */

// เลี้ยว 90 องศาที่จุดตัด  direction = "RIGHT" หรือ "LEFT"
void turn90(String direction) {
  moveFor();
  delay(NUDGE_TURN);
  stopRobot();
  if (direction == "RIGHT") { turnRight90(); }
  else                      { turnLeft90();  }
  stopRobot();
  leaveGride();
  clearPid();
}

// หยิบวัตถุที่ปลายเส้น แล้วกลับหลังหัน 180 องศา
void keep_item(String direction) {
  moveFor();
  delay(NUDGE_PICK);
  stopRobot();
  delay(500);
  keepup_object();
  if (direction == "RIGHT") { turnRight180(); }
  else                      { turnLeft180();  }
  stopRobot();
  leaveGride();
  clearPid();
}

// วางวัตถุที่ปลายเส้น
//   "RIGHT" / "LEFT" = วางแล้วยกแขนสูงและกลับหลังหัน 180 องศา
//   "STOP"           = วางแล้วอยู่กับที่ (ใช้กับจุดสุดท้ายของภารกิจ)
void place_item(String direction) {
  moveFor();
  delay(NUDGE_PLACE);
  stopRobot();
  delay(500);
  put_object();
  if (direction == "RIGHT") {
    arm_over_head();  //ยกแขนสูง กันไปเกี่ยวของที่เพิ่งวาง
    turnRight180();
  } else if (direction == "LEFT") {
    arm_over_head();  //ยกแขนสูง
    turnLeft180();
  }
  stopRobot();
  put_object();       //เอาแขนกลับท่าปกติ (ลงต่ำ + คลายหนีบ)
  if (direction != "STOP") { leaveGride(); }
  clearPid();
}

/* หลังเลี้ยวเสร็จ หุ่นอาจยังคร่อมเส้นตัดเดิมอยู่  ถ้าปล่อยไว้ countGrid()
 * จะนับจุดเดิมซ้ำ (และ while(checkGrid()) ข้างในจะค้าง เพราะล้อถูกเบรกอยู่)
 * จึงคลานไปข้างหน้าช้า ๆ จนพ้นเส้นตัดก่อน แล้วค่อยกลับไปเดินตามเส้นต่อ
 */
void leaveGride() {
  unsigned long t0 = millis();
  sp = 60;
  while (checkGrid() && millis() - t0 < 800) { moveFor(); }
  stopRobot();
}
