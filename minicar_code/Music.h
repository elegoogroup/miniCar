//基础音符
#define Note1_0 -1
#define Note1_1 262
#define Note1_2 294
#define Note1_3 330
#define Note1_4 350
#define Note1_5 393
#define Note1_6 441
#define Note1_7 495

#define NOTE1_LEN 32 //音符长度

//音符谱
int note1[] = {
    Note1_1, Note1_2, Note1_3, Note1_1, Note1_1,
    Note1_2, Note1_3, Note1_1, Note1_3, Note1_4,
    Note1_5, Note1_3, Note1_4, Note1_5, Note1_5,
    Note1_6, Note1_5, Note1_4, Note1_3, Note1_1,
    Note1_5, Note1_6, Note1_5, Note1_4, Note1_3,
    Note1_1, Note1_1, Note1_5, Note1_1, Note1_1,
    Note1_5, Note1_1};

//节拍谱
float beat1[] = {
    1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,
    2, 1, 1, 2, 0.75,
    0.25, 0.75, 0.25, 1, 1,
    0.75, 0.25, 0.75, 0.25, 1,
    1, 1, 1, 2, 1,
    1, 2};

void play1()
{
    for (int i = 0; i < NOTE1_LEN; i++)
    {
        tone(BEEP_PIN, note1[i]);
        delay(400 * beat1[i]);
        noTone(BEEP_PIN);
    }
}
