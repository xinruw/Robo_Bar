
/*
* Description: controller for the PR2
*/

/***************************** Header Files ********************************/
#include <webots/robot.h>
#include <webots/device.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
#include <webots/inertial_unit.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"


/*************************** Speech Recognition ****************************/
#define FRAME_LEN   640 
#define BUFFER_SIZE 4096

/* Upload User words */
static int upload_userwords()
{
    char*           userwords   =   NULL;
    size_t          len         =   0;
    size_t          read_len    =   0;
    FILE*           fp          =   NULL;
    int             ret         =   -1;

    fp = fopen("userwords.txt", "rb");
    if (NULL == fp)                                     
    {
        printf("\nopen [userwords.txt] failed! \n");
        goto upload_exit;
    }

    fseek(fp, 0, SEEK_END);
    len = ftell(fp); 
    fseek(fp, 0, SEEK_SET);                     
    
    userwords = (char*)malloc(len + 1);
    if (NULL == userwords)
    {
        printf("\nout of memory! \n");
        goto upload_exit;
    }

    read_len = fread((void*)userwords, 1, len, fp); 
    if (read_len != len)
    {
        printf("\nread [userwords.txt] failed!\n");
        goto upload_exit;
    }
    userwords[len] = '\0';
    
    MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); //é?′?ó??§′ê±í
    if (MSP_SUCCESS != ret)
    {
        printf("\nMSPUploadData failed ! errorCode: %d \n", ret);
        goto upload_exit;
    }
    
upload_exit:
    if (NULL != fp)
    {
        fclose(fp);
        fp = NULL;
    }   
    if (NULL != userwords)
    {
        free(userwords);
        userwords = NULL;
    }
    
    return ret;
}

static void show_result(char *string, char is_over)
{
    printf("\rResult: [ %s ]", string);
    if(is_over)
        putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
    if (result) {
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);
        if (left < size) {
            g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
            if (g_result)
                g_buffersize += BUFFER_SIZE;
            else {
                printf("mem alloc failed\n");
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}
void on_speech_begin()
{
    if (g_result)
    {
        free(g_result);
    }
    g_result = (char*)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);

    printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
        printf("\nSpeaking done \n");
    else
        printf("\nRecognizer error %d\n", reason);
}

/* demo send audio data from a file */
static void demo_file(const char* audio_file, const char* session_begin_params)
{
    int errcode = 0;
    FILE*   f_pcm = NULL;
    char*   p_pcm = NULL;
    unsigned long   pcm_count = 0;
    unsigned long   pcm_size = 0;
    unsigned long   read_size = 0;
    struct speech_rec iat;
    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end
    };

    if (NULL == audio_file)
        goto iat_exit;

    f_pcm = fopen(audio_file, "rb");
    if (NULL == f_pcm)
    {
        printf("\nopen [%s] failed! \n", audio_file);
        goto iat_exit;
    }

    fseek(f_pcm, 0, SEEK_END);
    pcm_size = ftell(f_pcm);
    fseek(f_pcm, 0, SEEK_SET);

    p_pcm = (char *)malloc(pcm_size);
    if (NULL == p_pcm)
    {
        printf("\nout of memory! \n");
        goto iat_exit;
    }

    read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm);
    if (read_size != pcm_size)
    {
        printf("\nread [%s] error!\n", audio_file);
        goto iat_exit;
    }

    errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
    if (errcode) {
        printf("speech recognizer init failed : %d\n", errcode);
        goto iat_exit;
    }

    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("\nsr_start_listening failed! error code:%d\n", errcode);
        goto iat_exit;
    }

    while (1)
    {
        unsigned int len = 10 * FRAME_LEN; /* 200ms audio */
        int ret = 0;

        if (pcm_size < 2 * len)
            len = pcm_size;
        if (len <= 0)
            break;

        ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len);

        if (0 != ret)
        {
            printf("\nwrite audio data failed! error code:%d\n", ret);
            goto iat_exit;
        }

        pcm_count += (long)len;
        pcm_size -= (long)len;      
    }

    errcode = sr_stop_listening(&iat);
    if (errcode) {
        printf("\nsr_stop_listening failed! error code:%d \n", errcode);
        goto iat_exit;
    }

iat_exit:
    if (NULL != f_pcm)
    {
        fclose(f_pcm);
        f_pcm = NULL;
    }
    if (NULL != p_pcm)
    {
        free(p_pcm);
        p_pcm = NULL;
    }

    sr_stop_listening(&iat);
    sr_uninit(&iat);
}

/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
    int errcode;
    int i = 0;

    struct speech_rec iat;

    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end
    };

    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("start listen failed %d\n", errcode);
    }
    /* demo 15 seconds recording */
    while(i++ < 15)
        sleep(1);
    errcode = sr_stop_listening(&iat);
    if (errcode) {
        printf("stop listening failed %d\n", errcode);
    }

    sr_uninit(&iat);
}

/***************************** String Parsing ******************************/

int getfoodID(char str[])
{
    int foodnum = 0;
    /*
    #define LEFT_WATER 1
    #define RIGHT_WATER 2
    #define BEER 3
    #define DOUBLE_BEER 4
    */ 
    const char *food1 = "左边的矿泉水";
    const char *food2 = "右边的矿泉水";
    const char *food3 = "一瓶啤酒";
    const char *food4 = "两瓶啤酒";
    char *p1 = strstr(str, food1);
    char *p2 = strstr(str, food2);
    char *p3 = strstr(str, food3);
    char *p4 = strstr(str, food4);
    if (p1 != NULL)
    {
        foodnum = 1;
    }
    else if (p2 != NULL)
    {
        foodnum = 2;
    }
    else if (p3 != NULL)
    {
        foodnum = 3;
    }
    else if (p4 != NULL)
    {
        foodnum = 4;
    }
    return foodnum;
}

int getclientID(char str[])
{
    int clientnum = 0;
    const char *num1 = "一号";
    const char *num2 = "二号";
    const char *num3 = "三号";
    const char *num4 = "四号";
    const char *num5 = "五号";
    const char *num6 = "六号";
    const char *num7 = "七号";
    const char *num11 = "1号";
    const char *num12 = "2号";
    const char *num13 = "3号";
    const char *num14 = "4号";
    const char *num15 = "5号";
    const char *num16 = "6号";
    const char *num17 = "7号";
    char *pp1 = strstr(str, num1);
    char *pp2 = strstr(str, num2);
    char *pp3 = strstr(str, num3);
    char *pp4 = strstr(str, num4);
    char *pp5 = strstr(str, num5);
    char *pp6 = strstr(str, num6);
    char *pp7 = strstr(str, num7);
    char *pp11 = strstr(str, num11);
    char *pp12 = strstr(str, num12);
    char *pp13 = strstr(str, num13);
    char *pp14 = strstr(str, num14);
    char *pp15 = strstr(str, num15);
    char *pp16 = strstr(str, num16);
    char *pp17 = strstr(str, num17);
    if (pp1 != NULL || pp11 != NULL)
    {
        clientnum = 1;
    }
    else if (pp2 != NULL || pp12 != NULL)
    {
        clientnum = 2;
    }
    else if (pp3 != NULL || pp13 != NULL)
    {
        clientnum = 3;
    }
    else if (pp4 != NULL || pp14 != NULL)
    {
        clientnum = 4;
    }
    else if (pp5 != NULL || pp15 != NULL)
    {
        clientnum = 5;
    }
    else if (pp6 != NULL || pp16 != NULL)
    {
        clientnum = 6;
    }
    else if (pp7 != NULL || pp17 != NULL)
    {
        clientnum = 7;
    }
    else
    {
        clientnum = 0;
    }
    return clientnum;
}

/**************************** Path Planning *********************************/
#define TURE 1
#define FAULT 0

//约定：0是可走的，1表示障碍物不可走，2表示起点，3表示终点，4表示路径
#define int_0 0
#define int_1 1
#define int_2 2
#define int_3 3
#define int_4 4

//#define M_PI 3.14159265358979323846 

// object and human constants
#define GROUND_LENGTH 10 /* x方向都是length，z方向都是width */
#define GROUND_WIDTH 10
#define PR2_LENGTH 0.5
#define BAR_LENGTH 0.4
#define BAR_WIDTH 7
#define CABINET_WIDTH 10
#define CABINET_LENGTH 0.7
#define TABLE_LENGTH 1.4
#define TABLE_WIDTH 1.4
#define MAX_TABEL_NUM 4
#define HUMAN_RADIUS 0.25
#define MAX_HUMAN_NUM 2
#define CHAIR_LENGTH 0.5
#define CHAIR_WIDTH 0.5
#define MAX_CHAIR_NUM 8
#define SQUARE_LENGTH 0.15
#define M_W  (int)(GROUND_LENGTH / SQUARE_LENGTH + 1)
#define M_H  (int)(GROUND_WIDTH / SQUARE_LENGTH + 1)
#define EDGE_SQUARE_NUM  (int)(0.5*PR2_LENGTH / SQUARE_LENGTH + 1) // 需要再objects边缘空出来的方块数 

typedef struct LNode {
    int data;    //对应数组中的数值
    int F;   //F = G + H;
    int G;   //G：从起点 A 移动到指定方格的移动代价，沿着到达该方格而生成的路径
    int H;   //H：从指定的方格移动到终点 B 的估算成本
    int x, y;   //对应数组中的坐标
    bool OPen_flag;  //在开放列表中为1，不在为0
    bool Close_flag;  //在关闭列表中为1，不在为0
    struct LNode* next;                    //用于链表排序
    struct LNode* path_parent;            //用于最终找到的路径从EndLNode往前寻找父节点，可以遍历整个链
    struct LNode* path_child;       //用于最终找到的路径从StartNode往后寻找子节点，可以遍历整个链
}LNode, *LinkList;

typedef struct LNode2 {
    double angle;           // 旋转角度
    double distance;          // 走的距离
    double orientation;         // 当前面向的角度，以x轴为极坐标轴
    struct LNode2* path_parent;     // 用于最终找到的路径从EndLNode往前寻找父节点，可以遍历整个链
    struct LNode2* path_child;      // 用于最终找到的路径从StartNode往后寻找子节点，可以遍历整个链
} LNode2;

// 全局变量
double table_coord[4][2];
double chair_coord[8][2];
double human_coord[2][2];
double bar_coord[2];
double cabinet_coord[2];
int    map[M_H][M_W];
double start_x, start_z, start_angle;
double end_x, end_z, end_angle;
int    guest_id, order_type;
// x, z, angle
double start_state[5][3] = { {0, 0, 0}, { 3.37756, -0.470738, 0 },{ 3.52615, 0.0234065, 0 },{ 3.66575, 0.16257, 0 },{ 3.46382, 0.0173562,0 } };
double end_state[8][3] = { {0,0,0}, {1.11779,2.64327,M_PI},{ -1.69258,2.66008,M_PI },{ -3.01883,1.36438,-M_PI / 2 },
    { -1.74688,-2.44716,M_PI },{ 1.11288,-2.41292,M_PI },{ 3.29782, 0.521792, M_PI },{ 3.29782, -2.4165,M_PI } };

// 新坐标
void get_coord() {
    bar_coord[0] = 2.21788 + 0.5*GROUND_LENGTH; bar_coord[1] = 0.111672 + 0.5*GROUND_WIDTH;

    cabinet_coord[0] = 4.55771 + 0.5*GROUND_LENGTH; cabinet_coord[1] = 0.104364 + 0.5*GROUND_WIDTH;

    table_coord[0][0] = -0.190068 + 0.5*GROUND_LENGTH; table_coord[0][1] = 2.61561 + 0.5*GROUND_WIDTH;
    table_coord[1][0] = -2.93322 + 0.5*GROUND_LENGTH; table_coord[1][1] = 2.66904 + 0.5*GROUND_WIDTH;
    table_coord[2][0] = -2.9895 + 0.5*GROUND_LENGTH; table_coord[2][1] = -2.42086 + 0.5*GROUND_WIDTH;
    table_coord[3][0] = -0.133898 + 0.5*GROUND_LENGTH; table_coord[3][1] = -2.42593 + 0.5*GROUND_WIDTH;

    chair_coord[0][0] = -0.186851 + 0.5*GROUND_LENGTH; chair_coord[0][1] = 3.8127 + 0.5*GROUND_WIDTH;
    chair_coord[1][0] = -3.04576 + 0.5*GROUND_LENGTH; chair_coord[1][1] = 3.86688 + 0.5*GROUND_WIDTH;
    chair_coord[2][0] = -4.10478 + 0.5*GROUND_LENGTH; chair_coord[2][1] = 2.71227 + 0.5*GROUND_WIDTH;
    chair_coord[3][0] = -2.96899 + 0.5*GROUND_LENGTH; chair_coord[3][1] = -1.25405 + 0.5*GROUND_WIDTH;
    chair_coord[4][0] = -4.18004 + 0.5*GROUND_LENGTH; chair_coord[4][1] = -2.47152 + 0.5*GROUND_WIDTH;
    chair_coord[5][0] = -2.9901 + 0.5*GROUND_LENGTH; chair_coord[5][1] = -3.57853 + 0.5*GROUND_WIDTH;
    chair_coord[6][0] = -0.100296 + 0.5*GROUND_LENGTH; chair_coord[6][1] = -1.19059 + 0.5*GROUND_WIDTH;
    chair_coord[7][0] = -0.16247 + 0.5*GROUND_LENGTH; chair_coord[7][1] = -3.62473 + 0.5*GROUND_WIDTH;

    human_coord[0][0] = 1.6573 + 0.5*GROUND_LENGTH; human_coord[0][1] = 0.761835 + 0.5*GROUND_WIDTH;
    human_coord[1][0] = 1.6573 + 0.5*GROUND_LENGTH; human_coord[1][1] = -2.18816 + 0.5*GROUND_WIDTH;
}

// 得到二维障碍物地图
void get_map() {
    get_coord();

    int z1, z2, x1, x2;

    /* 0：没有障碍物  1：有障碍物 */
    for (int i = 0; i < M_H; ++i)
        for (int j = 0; j < M_W; ++j) {
            map[i][j] = 0;
        }

    /* 墙的边缘要留出来 */
    for (int i = 0; i < EDGE_SQUARE_NUM; ++i) {
        for (int j = 0; j < M_H; ++j) {
            map[j][i] = 1;
            map[j][M_W - 1 - i] = 1;
        }
        for (int j = 0; j < M_W; ++j) {
            map[i][j] = 1;
            map[M_H - 1 - i][j] = 1;
        }
    }

    /* 吧台的边缘 */
    printf("bar(%f,%f)\n", bar_coord[0], bar_coord[1]);
    z1 = (bar_coord[1] - 0.5*BAR_WIDTH) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
    z2 = (bar_coord[1] + 0.5*BAR_WIDTH) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
    x1 = (bar_coord[0] - 0.5*BAR_LENGTH) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
    x2 = (bar_coord[0] + 0.5*BAR_LENGTH) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
    printf("z1=%d,z2=%d,x1=%d,x2=%d\n", z1, z2, x1, x2);
    for (int i = (z1 < 0 ? 0 : z1); i < (z2 >= M_H ? M_H - 1 : z2); ++i) {
        for (int j = (x1 < 0 ? 0 : x1); j < (x2 >= M_H ? M_H - 1 : x2); ++j) {
            map[i][j] = 1;
        }
    }

    /* 柜子的边缘 */
    z1 = (cabinet_coord[1] - 0.5*CABINET_WIDTH) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
    z2 = (cabinet_coord[1] + 0.5*CABINET_WIDTH) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
    x1 = (cabinet_coord[0] - 0.5*CABINET_LENGTH) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
    x2 = (cabinet_coord[0] + 0.5*CABINET_LENGTH) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
    for (int i = (z1 < 0 ? 0 : z1); i < (z2 >= M_H ? M_H - 1 : z2); ++i) {
        for (int j = (x1 < 0 ? 0 : x1); j < (x2 >= M_H ? M_H - 1 : x2); ++j) {
            map[i][j] = 1;
        }
    }

    /* 桌子的边缘 */
    for (int k = 0; k < MAX_TABEL_NUM; ++k) {
        z1 = (table_coord[k][1] - 0.5*TABLE_WIDTH) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
        z2 = (table_coord[k][1] + 0.5*TABLE_WIDTH) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
        x1 = (table_coord[k][0] - 0.5*TABLE_LENGTH) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
        x2 = (table_coord[k][0] + 0.5*TABLE_LENGTH) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
        for (int i = (z1 < 0 ? 0 : z1); i < (z2 >= M_H ? M_H - 1 : z2); ++i) {
            for (int j = (x1 < 0 ? 0 : x1); j < (x2 >= M_H ? M_H - 1 : x2); ++j) {
                map[i][j] = 1;
            }
        }
    }

    /* 椅子的边缘 */
    for (int k = 0; k < MAX_CHAIR_NUM; ++k) {
        z1 = (chair_coord[k][1] - 0.5*CHAIR_WIDTH) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
        z2 = (chair_coord[k][1] + 0.5*CHAIR_WIDTH) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
        x1 = (chair_coord[k][0] - 0.5*CHAIR_LENGTH) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
        x2 = (chair_coord[k][0] + 0.5*CHAIR_LENGTH) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
        for (int i = (z1 < 0 ? 0 : z1); i < (z2 >= M_H ? M_H - 1 : z2); ++i) {
            for (int j = (x1 < 0 ? 0 : x1); j < (x2 >= M_H ? M_H - 1 : x2); ++j) {
                map[i][j] = 1;
            }
        }
    }

    /* 人的边缘 */
    for (int k = 0; k < MAX_HUMAN_NUM; ++k) {
        int x0 = human_coord[k][0] / SQUARE_LENGTH;
        int z0 = human_coord[k][1] / SQUARE_LENGTH;
        int x1 = (human_coord[k][0] - HUMAN_RADIUS) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
        int x2 = (human_coord[k][0] + HUMAN_RADIUS) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
        int z1 = (human_coord[k][1] - HUMAN_RADIUS) / SQUARE_LENGTH - EDGE_SQUARE_NUM - 1;
        int z2 = (human_coord[k][1] + HUMAN_RADIUS) / SQUARE_LENGTH + 1 + EDGE_SQUARE_NUM;
        for (int i = z1; i < z2; ++i) {
            for (int j = x1; j < x2; ++j) {
                if (sqrt((i - z0)*(i - z0) + (j - x0)*(j - x0))
                    <= (HUMAN_RADIUS / SQUARE_LENGTH) + EDGE_SQUARE_NUM)
                    map[i][j] = 1;
            }
        }
    }
}

// A*算法
LinkList InitList()
{
    LinkList L = (LinkList)malloc(sizeof(LNode));
    if (L == NULL)
    {
        printf("Defeat!");
        exit(1);
    }
    memset(L, 0, sizeof(LNode));
    return L;
}//LinkList()

LNode** malloc_Array2D()
{
    LNode** map = (LNode**)malloc(M_H * sizeof(LNode*) + M_H*M_W * sizeof(LNode));
    LNode* head = (LNode*)(map + M_H);
    for (int i = 0; i < M_H; ++i)
        map[i] = head + i*M_W;
    return map;
}

LNode** Translate_Array(int Array[][M_W])
{
    LNode **map = malloc_Array2D();
    for (int i = 0; i < M_H; ++i)
        for (int j = 0; j < M_W; ++j)
        {
            (map[i] + j)->data = Array[i][j];
            (map[i] + j)->G = 0;
            (map[i] + j)->H = 0;
            (map[i] + j)->F = 0;    //(map[i] + j)->G + (map[i] + j)->H;
            (map[i] + j)->x = i;
            (map[i] + j)->y = j;
            (map[i] + j)->Close_flag = 0;
            (map[i] + j)->OPen_flag = 0;
            (map[i] + j)->next = NULL;
            (map[i] + j)->path_parent = NULL;
            (map[i] + j)->path_child = NULL;
        }
    return map;
}//Translate_Array()

void free_Array2D(LNode **arr)
{
    free(arr);
}

void output(LNode** Array)  //二维数组的访问必须指明位数，否则编译器不能解析
{
    for (int i = 0; i < M_H; ++i)
    {
        for (int j = 0; j < M_W; ++j)
        {
            printf("%d\t", (Array[i] + j)->data);
        }
        printf("\n");
    }
}

// 参数改变 直接传入目标坐标
LNode* find_start_LNode(LNode** Arr, int x, int y)    //从数组中找到始点
{
    LNode* start_LNode = NULL;

    start_LNode = (Arr[x] + y);
    //起点H=0,G=0,F=0
    start_LNode->G = 0;
    start_LNode->H = 0;
    start_LNode->F = 0;        //起点，则默认所有值为0
    return start_LNode;        //返回节点
}
LNode* find_end_LNode(LNode** Arr, int x, int y)        //从数组中找到终点
{
    LNode* end_LNode = NULL;
    end_LNode = (*(Arr + x) + y);
    end_LNode->F = 0;
    end_LNode->G = 0;
    end_LNode->H = 0;
    return end_LNode;        //返回节点
}

int count_LNode_G(LNode* curLNode, LNode* aheadLNode)        //计算节点的G值
{
    if (curLNode->x == aheadLNode->y && curLNode->y == aheadLNode->y)
        return 0;
    if (aheadLNode->x - curLNode->x != 0 && aheadLNode->y - curLNode->y != 0)
        curLNode->G = aheadLNode->G + 18;
    else
        curLNode->G = aheadLNode->G + 10;
    return curLNode->G;
}
int count_LNode_H(LNode* curLNode, LNode* endLNode)        //计算节点的H值
{
    curLNode->H = abs(endLNode->x - curLNode->x) * 8 + abs(endLNode->y - curLNode->y) * 8;
    return curLNode->H;
}
int count_LNode_F(LNode* curLNode)        //计算节点的F值
{
    curLNode->F = curLNode->G + curLNode->H;
    return curLNode->F;
}

//按从小到大的顺序
void push_OpenList_Node(LinkList L, LNode *elem)
{
    LNode *p, *q;
    p = q = L;
    while (p->next != NULL && p->F < elem->F)
    {
        q = p;
        p = p->next;
    }
    if (p->F < elem->F) q = p;
    elem->next = q->next;
    q->next = elem;
    //插入成功，更改属性值OPen_flag = 1
    elem->OPen_flag = 1;
}
//返回开放列表中F值最小的节点
LNode* pop_OpenList_minNode(LinkList L_OpenList)
{
    LNode *elem = NULL;
    if (L_OpenList->next)        //为了安全，防止访问空指针
    {
        L_OpenList->next->OPen_flag = 0;
        elem = L_OpenList->next;
        L_OpenList->next = L_OpenList->next->next;
        elem->next = NULL;
    }
    else
        printf("have a NULL point in pop_OpenList_mimNode()");
    return elem;
}
//插入OpenList中F值最小的节点到CloseList中去
bool insert_Into_CloseList(LNode* min_Open, LinkList L_CloseList)
{
    //对于CloseList中的节点并不需要排序,采用头插法
    min_Open->next = L_CloseList->next;
    L_CloseList->next = min_Open;
    min_Open->Close_flag = 1;
    return TURE;
}

bool isExist_openList(LNode* curLNode)
{
    return curLNode->OPen_flag;
}
bool isExist_closeList(LNode* curLNode)
{
    return curLNode->Close_flag;
}
bool isobstacle(LNode* curLNode)
{
    if (curLNode->data == 1)
        return TURE;
    else
        return FAULT;
}
//该节点是否可以加入开放列表
bool isJoin(LNode* cur)
{
    if (cur->x > -1 && cur->y > -1)            //边界检测
    {
        if (!isExist_closeList(cur) && !isobstacle(cur))        //既不在关闭列表里，也不是障碍物
        {
            return TURE;
        }
        else
            return FAULT;
    }
    return FAULT;
}
void insert_open(LNode *Node, LNode* ahead, LNode* endLNode, LinkList open_list, LNode** Arr)
{
    if (isJoin(Node))
    {
        if (isExist_openList(Node))
        {
            if (Node->x - ahead->x != 0 && Node->y - ahead->y != 0) {
                if (Node->F > (ahead->F + 18))
                {
                    count_LNode_G(Node, ahead);
                    count_LNode_F(Node);        //H值没有改变，所以还是原来的值
                    Node->path_parent = ahead;        //也不用再插入
                }
            }
            else {
                if (Node->F > (ahead->F + 10))
                {
                    count_LNode_G(Node, ahead);
                    count_LNode_F(Node);        //H值没有改变，所以还是原来的值
                    Node->path_parent = ahead;        //也不用再插入
                }
            }
        }
        else {
            count_LNode_G(Node, ahead);
            count_LNode_H(Node, endLNode);
            count_LNode_F(Node);
            Node->path_parent = ahead;
            push_OpenList_Node(open_list, Node);
        }
    }
}

int around_x[8] = { 0,0,1,1,1,-1,-1,-1 };
int around_y[8] = { -1,1,0,-1,1,0,-1,1 };
void check_around_curNode(LNode* cur, LNode* endLNode, LinkList open_list, LNode** Arr)
{
    int x = cur->x;
    int y = cur->y;
    for (int i = 0; i < 8; i++)
    {
        if (x + around_x[i] < 0 || x + around_x[i] >= M_H || y + around_y[i] < 0 || y + around_y[i] >= M_W)
            continue;
        else
            insert_open(Arr[x + around_x[i]] + y + around_y[i], cur, endLNode, open_list, Arr);
    }

}

// 得到每一步的旋转角度和行走距离
LNode2* get_operation_sequence(LNode* startLNode, double startAngle, double endAngle) {
    if (!startLNode)
        return NULL;

    // 注意：
    // x是向下方向，y是向右方向
    // 角度逆时针是正，顺时针是负

    //设置起始节点
    LNode2* startLNode2 = (LNode2*)malloc(sizeof(LNode));
    startLNode2->angle = 0;
    startLNode2->distance = 0;
    startLNode2->orientation = startAngle + 0.5*M_PI;
    startLNode2->path_parent = NULL;
    startLNode2->path_child = NULL;
    if (startLNode->path_child == NULL)
        return NULL;

    // 一个LNode对应一个LNode2
    LNode2* tempLNode2 = startLNode2;
    LNode* tempLNode = startLNode;
    while (tempLNode->path_child != NULL) {
        // LNode2生成一个子节点
        tempLNode2->path_child = (LNode2*)malloc(sizeof(LNode));

        // child->path_parent
        tempLNode2->path_child->path_parent = tempLNode2;

        // child->orientation
        if (tempLNode->path_child->x == tempLNode->x) { // 否则tan会除以零
            tempLNode2->path_child->orientation = 0.5*M_PI;
            if (tempLNode->path_child->y < tempLNode->y) {
                tempLNode2->path_child->orientation = -0.5*M_PI;
            }
        }
        else {
            tempLNode2->path_child->orientation =
                atan(((double)tempLNode->path_child->y - (double)tempLNode->y) / ((double)tempLNode->path_child->x - (double)tempLNode->x));
        }
        // 对arctan[-pi/2,pi/2]的结果进行调整，使朝向在(-pi,pi]之间
        if ((tempLNode2->path_child->orientation < 0)
            && (tempLNode->path_child->y > tempLNode->y)) { // y坐标后大于前，向上走
            tempLNode2->path_child->orientation += M_PI;
        }
        else if ((tempLNode2->path_child->orientation > 0)
            && (tempLNode->path_child->x < tempLNode->x)) { // x坐标后小于前，向下走
            tempLNode2->path_child->orientation -= M_PI;
        }
        else if ((tempLNode2->path_child->orientation == 0)
            && (tempLNode->path_child->x < tempLNode->x)) {
            tempLNode2->path_child->orientation = M_PI;
        }

        // child->(others)
        tempLNode2->path_child->angle = 0;
        tempLNode2->path_child->distance = 0;
        tempLNode2->path_child->path_child = NULL;

        // 旋转角度等于后面的减前面的
        tempLNode2->angle = tempLNode2->path_child->orientation - tempLNode2->orientation;
        // 如果绝对值超过pi，要进行调整
        if (tempLNode2->angle > M_PI)
            tempLNode2->angle -= 2 * M_PI;
        else if (tempLNode2->angle < -M_PI)
            tempLNode2->angle += 2 * M_PI;

        // 行走距离，从地图网格换算到世界坐标系下
        // 当前点坐标(x1,y1),后面一个点坐标(x2,y2)
        double x1 = tempLNode->x*SQUARE_LENGTH + 0.5*SQUARE_LENGTH;
        double x2 = tempLNode->path_child->x*SQUARE_LENGTH + 0.5*SQUARE_LENGTH;
        double y1 = tempLNode->y*SQUARE_LENGTH + 0.5*SQUARE_LENGTH;
        double y2 = tempLNode->path_child->y*SQUARE_LENGTH + 0.5*SQUARE_LENGTH;
        tempLNode2->distance = sqrt(pow(x1 - x2, 2)
            + pow(y1 - y2, 2));

        // temp到下一个节点
        tempLNode = tempLNode->path_child;
        tempLNode2 = tempLNode2->path_child;
    }

    // 尾节点
    tempLNode2->angle = endAngle + 0.5*M_PI - tempLNode2->orientation;
    if (tempLNode2->angle > M_PI)
        tempLNode2->angle -= 2 * M_PI;
    else if (tempLNode2->angle < -M_PI)
        tempLNode2->angle += 2 * M_PI;

    return startLNode2;
}

// 路径规划调用函数
LNode2* path_planning(double s_z, double s_x, double e_z, double e_x, double s_a, double e_a)
{
    get_map();

    int start_i = (s_z + 0.5*GROUND_WIDTH) / SQUARE_LENGTH;
    int start_j = (s_x + 0.5*GROUND_LENGTH) / SQUARE_LENGTH;
    int end_i = (e_z + 0.5*GROUND_WIDTH) / SQUARE_LENGTH;
    int end_j = (e_x + 0.5*GROUND_LENGTH) / SQUARE_LENGTH;

    LNode **ln_map = Translate_Array(map); //这里将数组的地图转为节点map的地图
                                           //output(ln_map, H, W);
    LinkList open_List = InitList();     //定义并初始化一个开放列表
    LinkList close_List = InitList();    //一个封闭列表
    LNode* startLNode = find_start_LNode(ln_map, start_i, start_j);
    LNode* endLNode = find_end_LNode(ln_map, end_i, end_j);

    LNode* curLNode = startLNode;        //当前节点=开始节点
    curLNode->G = 0;        //计算节点的三个值
    count_LNode_H(curLNode, endLNode);
    count_LNode_F(curLNode);
    push_OpenList_Node(open_List, curLNode);        //先将开始节点插入开放列表

    while (!(curLNode->x == end_i && curLNode->y == end_j))
    {
        //LNode *e = NULL;
        curLNode = pop_OpenList_minNode(open_List);
        insert_Into_CloseList(curLNode, close_List);
        //2、查看起点周围的点是否在开放列表里，不在加入，在检测经过该点F值是否最小等；
        check_around_curNode(curLNode, endLNode, open_List, ln_map);
    }

    endLNode->path_parent->path_child = endLNode;
    LNode* temp = endLNode;
    // 从最终节点出发，找出最终路径，让路径中前一点成为后一点的父节点，后一点成为前一点的子节点
    while (endLNode->path_parent)
    {
        endLNode->path_parent->path_child = temp;
        temp = endLNode->path_parent;
        endLNode->path_parent = endLNode->path_parent->path_parent;
    }

    // 从起始节点出发，遍历最终路径，去除一条直线上位于中心的多余节点
    temp = startLNode->path_child;
    while (temp->path_child && temp->path_parent)
    {
        if (temp->y - temp->path_child->y == 0 || temp->path_parent->y - temp->y == 0)
        {
            if (temp->y - temp->path_child->y == 0 && temp->path_parent->y - temp->y == 0)
            {
                temp->path_child->path_parent = temp->path_parent;
                temp->path_parent->path_child = temp->path_child;
            }
        }
        else
        {
            double k1 = 0.0 + (temp->x - temp->path_child->x) / (temp->y - temp->path_child->y);
            double k2 = 0.0 + (temp->path_parent->x - temp->x) / (temp->path_parent->y - temp->y);
            if (abs(k1 - k2) < 0.001)
            {
                temp->path_child->path_parent = temp->path_parent;
                temp->path_parent->path_child = temp->path_child;
            }
        }
        temp = temp->path_child;
    }

    // 从起始节点出发，遍历最终路径，使其平滑化
    temp = startLNode->path_child;
    while (temp->path_child && temp->path_parent)
    {
        bool flag = true;

        // y方向不变
        if (temp->path_parent->x == temp->path_child->x)
        {
            int now_x = temp->path_parent->x;
            int sou_y = temp->path_parent->y;
            int des_y = temp->path_child->y;
            if (sou_y < des_y)
            {
                for (int j = sou_y; j <= des_y; j++)
                {
                    if (map[now_x][j])
                    {
                        flag = false;
                        break;
                    }
                }
            }
            else
            {
                for (int j = des_y; j <= sou_y; j++)
                {
                    if (map[now_x][j])
                    {
                        flag = false;
                        break;
                    }
                }
            }
        }
        else
        {
            double k = (0.0 + (temp->path_parent->y - temp->path_child->y)) / (temp->path_parent->x - temp->path_child->x);
            int sign;
            if (temp->path_parent->x < temp->path_child->x)
                sign = 1;
            else
                sign = -1;

            int sou_y, now_x, des_y;
            sou_y = temp->path_parent->y;
            int i_sum = abs(temp->path_parent->x - temp->path_child->x);
            // temp->path_parent->x + (i + 0.5) * sign遍历所有的x方向格子,得到flag
            for (int i = 0; i < i_sum; i++)
            {
                now_x = temp->path_parent->x + i * sign;
                des_y = round(temp->path_parent->y + (i + 0.5) * sign * k);
                //判断当前x下是否有y对应的格子不可行
                if (sou_y < des_y)
                {
                    for (int j = sou_y; j <= des_y; j++)
                    {
                        if (map[now_x][j])
                        {
                            flag = false;
                            break;
                        }
                    }
                }
                else
                {
                    for (int j = des_y; j <= sou_y; j++)
                    {
                        if (map[now_x][j])
                        {
                            flag = false;
                            break;
                        }
                    }
                }
                if (!flag)
                    break;
                sou_y = des_y;
            }
            now_x = temp->path_child->x;
            des_y = temp->path_child->y;
            if (sou_y < des_y)
            {
                for (int j = sou_y; j <= des_y; j++)
                {
                    if (map[now_x][j])
                    {
                        flag = false;
                        break;
                    }
                }
            }
            else
            {
                for (int j = des_y; j <= sou_y; j++)
                {
                    if (map[now_x][j])
                    {
                        flag = false;
                        break;
                    }
                }
            }
        }

        if (flag)
        {
            temp->path_child->path_parent = temp->path_parent;
            temp->path_parent->path_child = temp->path_child;
        }
        temp = temp->path_child;
    }

    // 从起始节点出发，遍历最终路径并输出
    temp = startLNode;
    while (temp)
    {
        map[temp->x][temp->y] = 4;
        temp = temp->path_child;
    }

    LNode2 *temp2 = get_operation_sequence(startLNode, s_a, e_a);

    return temp2;
}

/***************************** Robot Control *******************************/
#define TIME_STEP 16

// PR2 constants
#define MAX_WHEEL_SPEED 3.0 // maximum velocity for the wheels [rad / s]
#define WHEELS_DISTANCE 0.4492 // distance between 2 caster wheels (the four wheels are located in square) [m]
#define SUB_WHEELS_DISTANCE 0.098 // distance between 2 sub wheels of a caster wheel [m]
#define WHEEL_RADIUS 0.08 // wheel radius

// function to check if a double is almost equal to another
#define TOLERANCE 0.05 // arbitrary value
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))

#define LEFT_WATER 1
#define RIGHT_WATER 2
#define BEER 3
#define DOUBLE_BEER 4

#define IN_BAR 5
#define OUT_BAR 6

//path planning
#ifndef APATH_H
#define APATH_H
#endif


// helper constants to distinguish the motors
enum { FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL };
enum { FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION };
enum { SHOULDER_ROLL, SHOULDER_LIFT, UPPER_ARM_ROLL, ELBOW_LIFT, FOREARM_ROLL, WRIST_LIFT, WRIST_ROLL };
enum { LEFT_FINGER, RIGHT_FINGER, LEFT_TIP, RIGHT_TIP };

// PR2 motors and their sensors
static WbDeviceTag wheel_motors[8];
static WbDeviceTag wheel_sensors[8];
static WbDeviceTag rotation_motors[4];
static WbDeviceTag rotation_sensors[4];
static WbDeviceTag left_arm_motors[7];
static WbDeviceTag left_arm_sensors[7];
static WbDeviceTag right_arm_motors[7];
static WbDeviceTag right_arm_sensors[7];
static WbDeviceTag right_finger_motors[4];
static WbDeviceTag right_finger_sensors[4];
static WbDeviceTag left_finger_motors[4];
static WbDeviceTag left_finger_sensors[4];
static WbDeviceTag head_tilt_motor;
static WbDeviceTag torso_motor;
static WbDeviceTag torso_sensor;

// PR2 sensor devices
static WbDeviceTag left_finger_contact_sensors[2];
static WbDeviceTag right_finger_contact_sensors[2];
static WbDeviceTag imu_sensor;
static WbDeviceTag wide_stereo_l_stereo_camera_sensor;
static WbDeviceTag wide_stereo_r_stereo_camera_sensor;
static WbDeviceTag high_def_sensor;
static WbDeviceTag r_forearm_cam_sensor;
static WbDeviceTag l_forearm_cam_sensor;
static WbDeviceTag laser_tilt;
static WbDeviceTag base_laser;



// Simpler step function
static void step() {
    if (wb_robot_step(TIME_STEP) == -1) {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

// Retrieve all the pointers to the PR2 devices
static void initialize_devices() {
    wheel_motors[FLL_WHEEL] = wb_robot_get_device("fl_caster_l_wheel_joint");
    wheel_motors[FLR_WHEEL] = wb_robot_get_device("fl_caster_r_wheel_joint");
    wheel_motors[FRL_WHEEL] = wb_robot_get_device("fr_caster_l_wheel_joint");
    wheel_motors[FRR_WHEEL] = wb_robot_get_device("fr_caster_r_wheel_joint");
    wheel_motors[BLL_WHEEL] = wb_robot_get_device("bl_caster_l_wheel_joint");
    wheel_motors[BLR_WHEEL] = wb_robot_get_device("bl_caster_r_wheel_joint");
    wheel_motors[BRL_WHEEL] = wb_robot_get_device("br_caster_l_wheel_joint");
    wheel_motors[BRR_WHEEL] = wb_robot_get_device("br_caster_r_wheel_joint");
    wheel_sensors[FLL_WHEEL] = wb_robot_get_device("fl_caster_l_wheel_joint_sensor");
    wheel_sensors[FLR_WHEEL] = wb_robot_get_device("fl_caster_r_wheel_joint_sensor");
    wheel_sensors[FRL_WHEEL] = wb_robot_get_device("fr_caster_l_wheel_joint_sensor");
    wheel_sensors[FRR_WHEEL] = wb_robot_get_device("fr_caster_r_wheel_joint_sensor");
    wheel_sensors[BLL_WHEEL] = wb_robot_get_device("bl_caster_l_wheel_joint_sensor");
    wheel_sensors[BLR_WHEEL] = wb_robot_get_device("bl_caster_r_wheel_joint_sensor");
    wheel_sensors[BRL_WHEEL] = wb_robot_get_device("br_caster_l_wheel_joint_sensor");
    wheel_sensors[BRR_WHEEL] = wb_robot_get_device("br_caster_r_wheel_joint_sensor");

    rotation_motors[FL_ROTATION] = wb_robot_get_device("fl_caster_rotation_joint");
    rotation_motors[FR_ROTATION] = wb_robot_get_device("fr_caster_rotation_joint");
    rotation_motors[BL_ROTATION] = wb_robot_get_device("bl_caster_rotation_joint");
    rotation_motors[BR_ROTATION] = wb_robot_get_device("br_caster_rotation_joint");
    rotation_sensors[FL_ROTATION] = wb_robot_get_device("fl_caster_rotation_joint_sensor");
    rotation_sensors[FR_ROTATION] = wb_robot_get_device("fr_caster_rotation_joint_sensor");
    rotation_sensors[BL_ROTATION] = wb_robot_get_device("bl_caster_rotation_joint_sensor");
    rotation_sensors[BR_ROTATION] = wb_robot_get_device("br_caster_rotation_joint_sensor");

    left_arm_motors[SHOULDER_ROLL] = wb_robot_get_device("l_shoulder_pan_joint");
    left_arm_motors[SHOULDER_LIFT] = wb_robot_get_device("l_shoulder_lift_joint");
    left_arm_motors[UPPER_ARM_ROLL] = wb_robot_get_device("l_upper_arm_roll_joint");
    left_arm_motors[ELBOW_LIFT] = wb_robot_get_device("l_elbow_flex_joint");
    left_arm_motors[FOREARM_ROLL] = wb_robot_get_device("l_forearm_roll_joint");
    left_arm_motors[WRIST_LIFT] = wb_robot_get_device("l_wrist_flex_joint");
    left_arm_motors[WRIST_ROLL] = wb_robot_get_device("l_wrist_roll_joint");
    left_arm_sensors[SHOULDER_ROLL] = wb_robot_get_device("l_shoulder_pan_joint_sensor");
    left_arm_sensors[SHOULDER_LIFT] = wb_robot_get_device("l_shoulder_lift_joint_sensor");
    left_arm_sensors[UPPER_ARM_ROLL] = wb_robot_get_device("l_upper_arm_roll_joint_sensor");
    left_arm_sensors[ELBOW_LIFT] = wb_robot_get_device("l_elbow_flex_joint_sensor");
    left_arm_sensors[FOREARM_ROLL] = wb_robot_get_device("l_forearm_roll_joint_sensor");
    left_arm_sensors[WRIST_LIFT] = wb_robot_get_device("l_wrist_flex_joint_sensor");
    left_arm_sensors[WRIST_ROLL] = wb_robot_get_device("l_wrist_roll_joint_sensor");

    right_arm_motors[SHOULDER_ROLL] = wb_robot_get_device("r_shoulder_pan_joint");
    right_arm_motors[SHOULDER_LIFT] = wb_robot_get_device("r_shoulder_lift_joint");
    right_arm_motors[UPPER_ARM_ROLL] = wb_robot_get_device("r_upper_arm_roll_joint");
    right_arm_motors[ELBOW_LIFT] = wb_robot_get_device("r_elbow_flex_joint");
    right_arm_motors[FOREARM_ROLL] = wb_robot_get_device("r_forearm_roll_joint");
    right_arm_motors[WRIST_LIFT] = wb_robot_get_device("r_wrist_flex_joint");
    right_arm_motors[WRIST_ROLL] = wb_robot_get_device("r_wrist_roll_joint");
    right_arm_sensors[SHOULDER_ROLL] = wb_robot_get_device("r_shoulder_pan_joint_sensor");
    right_arm_sensors[SHOULDER_LIFT] = wb_robot_get_device("r_shoulder_lift_joint_sensor");
    right_arm_sensors[UPPER_ARM_ROLL] = wb_robot_get_device("r_upper_arm_roll_joint_sensor");
    right_arm_sensors[ELBOW_LIFT] = wb_robot_get_device("r_elbow_flex_joint_sensor");
    right_arm_sensors[FOREARM_ROLL] = wb_robot_get_device("r_forearm_roll_joint_sensor");
    right_arm_sensors[WRIST_LIFT] = wb_robot_get_device("r_wrist_flex_joint_sensor");
    right_arm_sensors[WRIST_ROLL] = wb_robot_get_device("r_wrist_roll_joint_sensor");

    left_finger_motors[LEFT_FINGER] = wb_robot_get_device("l_gripper_l_finger_joint");
    left_finger_motors[RIGHT_FINGER] = wb_robot_get_device("l_gripper_r_finger_joint");
    left_finger_motors[LEFT_TIP] = wb_robot_get_device("l_gripper_l_finger_tip_joint");
    left_finger_motors[RIGHT_TIP] = wb_robot_get_device("l_gripper_r_finger_tip_joint");
    left_finger_sensors[LEFT_FINGER] = wb_robot_get_device("l_gripper_l_finger_joint_sensor");
    left_finger_sensors[RIGHT_FINGER] = wb_robot_get_device("l_gripper_r_finger_joint_sensor");
    left_finger_sensors[LEFT_TIP] = wb_robot_get_device("l_gripper_l_finger_tip_joint_sensor");
    left_finger_sensors[RIGHT_TIP] = wb_robot_get_device("l_gripper_r_finger_tip_joint_sensor");

    right_finger_motors[LEFT_FINGER] = wb_robot_get_device("r_gripper_l_finger_joint");
    right_finger_motors[RIGHT_FINGER] = wb_robot_get_device("r_gripper_r_finger_joint");
    right_finger_motors[LEFT_TIP] = wb_robot_get_device("r_gripper_l_finger_tip_joint");
    right_finger_motors[RIGHT_TIP] = wb_robot_get_device("r_gripper_r_finger_tip_joint");
    right_finger_sensors[LEFT_FINGER] = wb_robot_get_device("r_gripper_l_finger_joint_sensor");
    right_finger_sensors[RIGHT_FINGER] = wb_robot_get_device("r_gripper_r_finger_joint_sensor");
    right_finger_sensors[LEFT_TIP] = wb_robot_get_device("r_gripper_l_finger_tip_joint_sensor");
    right_finger_sensors[RIGHT_TIP] = wb_robot_get_device("r_gripper_r_finger_tip_joint_sensor");

    head_tilt_motor = wb_robot_get_device("head_tilt_joint");
    torso_motor = wb_robot_get_device("torso_lift_joint");
    torso_sensor = wb_robot_get_device("torso_lift_joint_sensor");

    left_finger_contact_sensors[LEFT_FINGER] = wb_robot_get_device("l_gripper_l_finger_tip_contact_sensor");
    left_finger_contact_sensors[RIGHT_FINGER] = wb_robot_get_device("l_gripper_r_finger_tip_contact_sensor");
    right_finger_contact_sensors[LEFT_FINGER] = wb_robot_get_device("r_gripper_l_finger_tip_contact_sensor");
    right_finger_contact_sensors[RIGHT_FINGER] = wb_robot_get_device("r_gripper_r_finger_tip_contact_sensor");

    imu_sensor = wb_robot_get_device("imu_sensor");

    wide_stereo_l_stereo_camera_sensor = wb_robot_get_device("wide_stereo_l_stereo_camera_sensor");
    wide_stereo_r_stereo_camera_sensor = wb_robot_get_device("wide_stereo_r_stereo_camera_sensor");
    high_def_sensor = wb_robot_get_device("high_def_sensor");
    r_forearm_cam_sensor = wb_robot_get_device("r_forearm_cam_sensor");
    l_forearm_cam_sensor = wb_robot_get_device("l_forearm_cam_sensor");
    laser_tilt = wb_robot_get_device("laser_tilt");
    base_laser = wb_robot_get_device("base_laser");
}

// enable the robot devices
static void enable_devices() {
    int i = 0;
    for (i = 0; i < 8; ++i) {
        wb_position_sensor_enable(wheel_sensors[i], TIME_STEP);
        // init the motors for speed control
        wb_motor_set_position(wheel_motors[i], INFINITY);
        wb_motor_set_velocity(wheel_motors[i], 0.0);
    }

    for (i = 0; i < 4; ++i)
        wb_position_sensor_enable(rotation_sensors[i], TIME_STEP);

    for (i = 0; i < 2; ++i) {
        wb_touch_sensor_enable(left_finger_contact_sensors[i], TIME_STEP);
        wb_touch_sensor_enable(right_finger_contact_sensors[i], TIME_STEP);
    }

    for (i = 0; i < 4; ++i) {
        wb_position_sensor_enable(left_finger_sensors[i], TIME_STEP);
        wb_position_sensor_enable(right_finger_sensors[i], TIME_STEP);
    }

    for (i = 0; i < 7; ++i) {
        wb_position_sensor_enable(left_arm_sensors[i], TIME_STEP);
        wb_position_sensor_enable(right_arm_sensors[i], TIME_STEP);
    }

    wb_position_sensor_enable(torso_sensor, TIME_STEP);

    // the following devices are not used in this simulation.
    // wb_inertial_unit_enable(imu_sensor, TIME_STEP);
    wb_camera_enable(wide_stereo_l_stereo_camera_sensor, TIME_STEP);
    wb_camera_enable(wide_stereo_r_stereo_camera_sensor, TIME_STEP);
    wb_camera_enable(high_def_sensor, TIME_STEP);
    wb_camera_enable(r_forearm_cam_sensor, TIME_STEP);
    wb_camera_enable(l_forearm_cam_sensor, TIME_STEP);
    // wb_lidar_enable(laser_tilt, TIME_STEP);
    // wb_lidar_enable(base_laser, TIME_STEP);
}

// set the speeds of the robot wheels
static void set_wheels_speeds(
    double fll, double flr, double frl, double frr,
    double bll, double blr, double brl, double brr
) {
    wb_motor_set_velocity(wheel_motors[FLL_WHEEL], fll);
    wb_motor_set_velocity(wheel_motors[FLR_WHEEL], flr);
    wb_motor_set_velocity(wheel_motors[FRL_WHEEL], frl);
    wb_motor_set_velocity(wheel_motors[FRR_WHEEL], frr);
    wb_motor_set_velocity(wheel_motors[BLL_WHEEL], bll);
    wb_motor_set_velocity(wheel_motors[BLR_WHEEL], blr);
    wb_motor_set_velocity(wheel_motors[BRL_WHEEL], brl);
    wb_motor_set_velocity(wheel_motors[BRR_WHEEL], brr);
}

static void set_wheels_speed(double speed) {
    set_wheels_speeds(speed, speed, speed, speed, speed, speed, speed, speed);
}

static void stop_wheels() {
    set_wheels_speeds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// enable/disable the torques on the wheels motors
static void enable_passive_wheels(bool enable) {
    static double torques[8] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    int i;
    if (enable) {
        for (i = 0; i < 8; ++i) {
            torques[i] = wb_motor_get_available_torque(wheel_motors[i]);
            wb_motor_set_available_torque(wheel_motors[i], 0.0);
        }
    }
    else {
        for (i = 0; i < 8; ++i)
            wb_motor_set_available_torque(wheel_motors[i], torques[i]);
    }
}

// Set the rotation wheels angles.
// If wait_on_feedback is true, the function is left when the rotational motors have reached their target positions.
static void set_rotation_wheels_angles(double fl, double fr, double bl, double br, bool wait_on_feedback) {
    if (wait_on_feedback) {
        stop_wheels();
        enable_passive_wheels(true);
    }

    wb_motor_set_position(rotation_motors[FL_ROTATION], fl);
    wb_motor_set_position(rotation_motors[FR_ROTATION], fr);
    wb_motor_set_position(rotation_motors[BL_ROTATION], bl);
    wb_motor_set_position(rotation_motors[BR_ROTATION], br);

    if (wait_on_feedback) {
        double target[4] = { fl, fr, bl, br };

        while (true) {
            bool all_reached = true;
            int i;
            for (i = 0; i < 4; ++i) {
                double current_position = wb_position_sensor_get_value(rotation_sensors[i]);
                if (!ALMOST_EQUAL(current_position, target[i])) {
                    all_reached = false;
                    break;
                }
            }

            if (all_reached)
                break;
            else
                step();
        }

        enable_passive_wheels(false);
    }
}

// High level function to rotate the robot around itself of a given angle [rad]
// Note: the angle can be negative
static void robot_rotate(double angle) {
    stop_wheels();

    set_rotation_wheels_angles(3.0 * M_PI_4, M_PI_4, -3.0 * M_PI_4, -M_PI_4, true);
    const double max_wheel_speed = angle > 0 ? MAX_WHEEL_SPEED : -MAX_WHEEL_SPEED;
    set_wheels_speed(max_wheel_speed);

    double initial_wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
    double expected_travel_distance = fabs(angle * 0.5 * (WHEELS_DISTANCE + SUB_WHEELS_DISTANCE)); // expected travel distance done by the wheel

    while (true) {
        double wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
        double wheel0_travel_distance = fabs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position)); // travel distance done by the wheel

                                                                                                          //if (wheel0_travel_distance > expected_travel_distance)
        if (-wheel0_travel_distance + expected_travel_distance<0.001)
            break;

        // reduce the speed before reaching the target
        if (expected_travel_distance - wheel0_travel_distance < 0.015)
            set_wheels_speed(0.01 * max_wheel_speed);

        step();
    }

    // reset wheels
    stop_wheels();
    set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, true);
}

static void robot_rotate_right(double angle) {
    stop_wheels();

    set_rotation_wheels_angles(3.0 * M_PI_4, M_PI_4, -3.0 * M_PI_4, -M_PI_4, true);
    const double max_wheel_speed = angle > 0 ? -MAX_WHEEL_SPEED : MAX_WHEEL_SPEED;
    set_wheels_speed(max_wheel_speed);

    double initial_wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
    double expected_travel_distance = fabs(angle * 0.5 * (WHEELS_DISTANCE + SUB_WHEELS_DISTANCE)); // expected travel distance done by the wheel

    while (true) {
        double wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
        double wheel0_travel_distance = fabs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position)); // travel distance done by the wheel

                                                                                                          //if (wheel0_travel_distance > expected_travel_distance)
        if (-wheel0_travel_distance + expected_travel_distance<0.001)
            break;

        // reduce the speed before reaching the target
        if (expected_travel_distance - wheel0_travel_distance < 0.015) {
            set_wheels_speed(0.01 * max_wheel_speed);
        }

        step();
    }

    // reset wheels
    stop_wheels();
    set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, true);
}

// High level function to go forward for a given distance [m]
// Note: the distance can be negative
static void robot_go_forward(double distance) {
    double max_wheel_speed = distance > 0 ? MAX_WHEEL_SPEED : -MAX_WHEEL_SPEED;
    set_wheels_speed(max_wheel_speed);

    double initial_wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
    printf("%f", initial_wheel0_position);

    while (true) {
        double wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
        double wheel0_travel_distance = fabs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position)); // travel distance done by the wheel

                                                                                                          //if(wheel0_travel_distance>fabs(distance))
        if (-wheel0_travel_distance + fabs(distance)<0.001)
            break;

        // reduce the speed before reaching the target
        if (fabs(distance) - wheel0_travel_distance < 0.015)
            set_wheels_speed(0.01 * max_wheel_speed);

        step();
    }

    stop_wheels();
}

// Open or close the gripper.
// If wait_on_feedback is true, the gripper is stopped either when the target is reached,
//  or either when something has been gripped
static void set_gripper(bool left, bool open, double torqueWhenGripping, bool wait_on_feedback) {
    WbDeviceTag motors[4];
    motors[LEFT_FINGER] = left ? left_finger_motors[LEFT_FINGER] : right_finger_motors[LEFT_FINGER];
    motors[RIGHT_FINGER] = left ? left_finger_motors[RIGHT_FINGER] : right_finger_motors[RIGHT_FINGER];
    motors[LEFT_TIP] = left ? left_finger_motors[LEFT_TIP] : right_finger_motors[LEFT_TIP];
    motors[RIGHT_TIP] = left ? left_finger_motors[RIGHT_TIP] : right_finger_motors[RIGHT_TIP];

    WbDeviceTag sensors[4];
    sensors[LEFT_FINGER] = left ? left_finger_sensors[LEFT_FINGER] : right_finger_sensors[LEFT_FINGER];
    sensors[RIGHT_FINGER] = left ? left_finger_sensors[RIGHT_FINGER] : right_finger_sensors[RIGHT_FINGER];
    sensors[LEFT_TIP] = left ? left_finger_sensors[LEFT_TIP] : right_finger_sensors[LEFT_TIP];
    sensors[RIGHT_TIP] = left ? left_finger_sensors[RIGHT_TIP] : right_finger_sensors[RIGHT_TIP];

    WbDeviceTag contacts[2];
    contacts[LEFT_FINGER] = left ? left_finger_contact_sensors[LEFT_FINGER] : right_finger_contact_sensors[LEFT_FINGER];
    contacts[RIGHT_FINGER] = left ? left_finger_contact_sensors[RIGHT_FINGER] : right_finger_contact_sensors[RIGHT_FINGER];

    static bool firstCall = true;
    static double maxTorque = 0.0;
    if (firstCall) {
        maxTorque = wb_motor_get_available_torque(motors[LEFT_FINGER]);
        firstCall = false;
    }

    int i;
    for (i = 0; i < 4; ++i)
        wb_motor_set_available_torque(motors[i], maxTorque);

    if (open) {
        static const double targetOpenValue = 0.54;
        for (i = 0; i < 4; ++i)
            wb_motor_set_position(motors[i], targetOpenValue);

        if (wait_on_feedback) {
            while (!ALMOST_EQUAL(wb_position_sensor_get_value(sensors[LEFT_FINGER]), targetOpenValue))
                step();
        }
    }
    else {
        static const double targetCloseValue = 0.0;
        for (i = 0; i < 4; ++i)
            wb_motor_set_position(motors[i], targetCloseValue);

        if (wait_on_feedback) {
            // wait until the 2 touch sensors are fired or the target value is reached
            while (
                (wb_touch_sensor_get_value(contacts[LEFT_FINGER]) == 0.0 ||
                    wb_touch_sensor_get_value(contacts[RIGHT_FINGER]) == 0.0) &&
                !ALMOST_EQUAL(wb_position_sensor_get_value(sensors[LEFT_FINGER]), targetCloseValue)
                ) {
                step();
            }
            double current_position = wb_position_sensor_get_value(sensors[LEFT_FINGER]);
            for (i = 0; i < 4; ++i) {
                wb_motor_set_available_torque(motors[i], torqueWhenGripping);
                wb_motor_set_position(motors[i], fmax(0.0, 0.95 * current_position));
            }
        }
    }
}

// Set the right arm position (forward kinematics)
// If wait_on_feedback is enabled, the function is left when the target is reached.
static void set_right_arm_position(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift, double wrist_roll, bool wait_on_feedback) {
    wb_motor_set_position(right_arm_motors[SHOULDER_ROLL], shoulder_roll);
    wb_motor_set_position(right_arm_motors[SHOULDER_LIFT], shoulder_lift);
    wb_motor_set_position(right_arm_motors[UPPER_ARM_ROLL], upper_arm_roll);
    wb_motor_set_position(right_arm_motors[ELBOW_LIFT], elbow_lift);
    wb_motor_set_position(right_arm_motors[WRIST_ROLL], wrist_roll);

    if (wait_on_feedback) {
        while (
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[WRIST_ROLL]), wrist_roll)
            ) {
            step();
        }
    }
}

static void set_right_arm_position_upper(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift, double forearm_roll, double wrist_lift, double wrist_roll, bool wait_on_feedback) {
    wb_motor_set_position(right_arm_motors[SHOULDER_ROLL], shoulder_roll);
    wb_motor_set_position(right_arm_motors[SHOULDER_LIFT], shoulder_lift);
    wb_motor_set_position(right_arm_motors[UPPER_ARM_ROLL], upper_arm_roll);
    wb_motor_set_position(right_arm_motors[ELBOW_LIFT], elbow_lift);
    wb_motor_set_position(right_arm_motors[FOREARM_ROLL], forearm_roll);
    wb_motor_set_position(right_arm_motors[WRIST_LIFT], wrist_lift);
    wb_motor_set_position(right_arm_motors[WRIST_ROLL], wrist_roll);

    if (wait_on_feedback) {
        while (
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[FOREARM_ROLL]), forearm_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[WRIST_LIFT]), wrist_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[WRIST_ROLL]), wrist_roll)
            ) {
            step();
        }
    }
}


// Idem for the left arm
static void set_left_arm_position(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift, double wrist_roll, bool wait_on_feedback) {
    wb_motor_set_position(left_arm_motors[SHOULDER_ROLL], shoulder_roll);
    wb_motor_set_position(left_arm_motors[SHOULDER_LIFT], shoulder_lift);
    wb_motor_set_position(left_arm_motors[UPPER_ARM_ROLL], upper_arm_roll);
    wb_motor_set_position(left_arm_motors[ELBOW_LIFT], elbow_lift);
    wb_motor_set_position(left_arm_motors[WRIST_ROLL], wrist_roll);

    if (wait_on_feedback) {
        while (
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[WRIST_ROLL]), wrist_roll)
            ) {
            step();
        }
    }
}


static void set_left_arm_position_upper(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift, double forearm_roll, double wrist_lift, double wrist_roll, bool wait_on_feedback) {
    wb_motor_set_position(left_arm_motors[SHOULDER_ROLL], shoulder_roll);
    wb_motor_set_position(left_arm_motors[SHOULDER_LIFT], shoulder_lift);
    wb_motor_set_position(left_arm_motors[UPPER_ARM_ROLL], upper_arm_roll);
    wb_motor_set_position(left_arm_motors[ELBOW_LIFT], elbow_lift);
    wb_motor_set_position(left_arm_motors[FOREARM_ROLL], forearm_roll);
    wb_motor_set_position(left_arm_motors[WRIST_LIFT], wrist_lift);
    wb_motor_set_position(left_arm_motors[WRIST_ROLL], wrist_roll);

    if (wait_on_feedback) {
        while (
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[FOREARM_ROLL]), forearm_roll) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[WRIST_LIFT]), wrist_lift) ||
            !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[WRIST_ROLL]), wrist_roll)
            ) {
            step();
        }
    }
}

// Set the torso height
// If wait_on_feedback is enabled, the function is left when the target is reached.
static void set_torso_height(double height, bool wait_on_feedback) {
    wb_motor_set_position(torso_motor, height);

    if (wait_on_feedback) {
        while (!ALMOST_EQUAL(wb_position_sensor_get_value(torso_sensor), height))
            step();
    }
}

// Convenient initial position
static void set_initial_position(int type) {
    set_left_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, false);
    set_right_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, false);

    set_gripper(false, true, 0.0, false);
    set_gripper(true, true, 0.0, false);

    switch (type)
    {
    case LEFT_WATER:
    {
        set_torso_height(0.1, true);
        break;
    }
    case RIGHT_WATER:
    {
        set_torso_height(0.1, true);
        break;
    }
    case BEER:
    {
        set_torso_height(0.06, true);
        break;
    }
    case DOUBLE_BEER:
    {
        set_torso_height(0.06, true);
        break;
    }
    }

}

void set_start_end_state(int order_type, int guest_id) {
    start_x = start_state[order_type][0];
    start_z = start_state[order_type][1];
    start_angle = start_state[order_type][2];

    end_x = end_state[guest_id][0];
    end_z = end_state[guest_id][1];
    end_angle = end_state[guest_id][2];
}

void go_back(double sz, double sx, double ez, double ex, double sa, double ea)
{
  LNode2* movement = path_planning(sz, sx,ez,ex,sa,ea);
  LNode2* start=movement;
  while(start!=NULL)
  {
    robot_rotate(start->angle);
    robot_go_forward(start->distance);
    start=start->path_child;
  }
  
}

void serve(int order_type, int guest_id) {

    LNode2 *movement, *start;

    switch (order_type) {
    case LEFT_WATER:
        /* fetch */
        robot_rotate_right(M_PI_4 * 2);
        robot_go_forward(0.5);
        robot_rotate_right(M_PI_4 * 2);
        set_right_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, true);
        robot_go_forward(0.15);
        //get the drink
        set_gripper(false, false, 20.0, true);
        // lift the arms
        set_right_arm_position(0.0, 0.5, 0.0, -1.0, 0.0, true);

        /* go */
        movement = path_planning(start_z, start_x, end_z, end_x, start_angle, end_angle);
        start = movement;
        while (start != NULL) {
            robot_rotate(start->angle);
            robot_go_forward(start->distance);
            start = start->path_child;
        }

        /* place */
        set_torso_height(0.3, true);
        set_right_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, true);
        set_gripper(false, true, 0.0, true);
        //robot_go_forward(-0.15);
        set_right_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, true);

        go_back(end_z,end_x,start_z,start_x,end_angle,start_angle+M_PI);

        break;

    case RIGHT_WATER:
        /* fetch */
        robot_rotate(M_PI);
        set_left_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, true);
        robot_go_forward(0.15);
        //get the drink
        set_gripper(true, false, 20.0, true);
        // lift the arms
        set_left_arm_position(0.0, 0.5, 0.0, -1.0, 0.0, true);

        /* go */
        movement = path_planning(start_z, start_x, end_z, end_x, start_angle, end_angle);
        start = movement;
        while (start != NULL) {
            robot_rotate(start->angle);
            robot_go_forward(start->distance);
            start = start->path_child;
        }

        /* place */
        set_torso_height(0.3, true);
        set_left_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, true);
        set_gripper(true, true, 0.0, true);
        //robot_go_forward(-0.15);
        set_left_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, true);
        set_right_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, true);

        go_back(end_z,end_x,start_z,start_x,end_angle,start_angle+M_PI);

        break;

    case BEER:
        /* fetch */
        robot_rotate(M_PI_4 * 2);
        robot_go_forward(0.15);
        robot_rotate(M_PI_4 * 2);
        set_right_arm_position_upper(0.0, -0.3536, 0.0, -0.7566, 3.1415, -1.5, -3.1415, true);
        robot_go_forward(0.3);
        //get the drink
        set_gripper(false, false, 20.0, true);
        // lift the arms
        set_right_arm_position_upper(0.0, -0.3536, 0.0, -0.7566, 3.1415, -1, -3.1415, true);

        /* go */
        movement = path_planning(start_z, start_x, end_z, end_x, start_angle, end_angle);
        start = movement;
        while (start != NULL) {
            robot_rotate(start->angle);
            robot_go_forward(start->distance);
            start = start->path_child;
        }

        /* place */
        set_torso_height(0.3, true);
        set_right_arm_position_upper(0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0, true);
        //move arm down
        set_right_arm_position_upper(0.0, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0, true);
        robot_go_forward(0.2);
        // open the grippers
        set_gripper(false, true, 0.0, true);
        //robot_go_forward(-0.15);
        set_right_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, true);

        go_back(end_z,end_x,start_z,start_x,end_angle,start_angle+M_PI);

        break;

    case DOUBLE_BEER:
        /* fetch */
        robot_rotate(M_PI);
        set_left_arm_position_upper(0.0, -0.3536, 0.0, -0.7566, 3.1415, -1.5, -3.1415, true);
        set_right_arm_position_upper(0.0, -0.3536, 0.0, -0.7566, 3.1415, -1.5, -3.1415, true);
        robot_go_forward(0.3);
        //get the drink
        set_gripper(true, false, 20.0, true);
        set_gripper(false, false, 20.0, true);
        // lift the arms
        set_left_arm_position_upper(0.0, -0.3536, 0.0, -0.7566, 3.1415, -1, -3.1415, true);
        set_right_arm_position_upper(0.0, -0.3536, 0.0, -0.7566, 3.1415, -1, -3.1415, true);

        robot_go_forward(-0.2);

        /* go */
        movement = path_planning(start_z, start_x, end_z, end_x, start_angle, end_angle);
        start = movement;
        while (start != NULL) {
            robot_rotate(start->angle);
            robot_go_forward(start->distance);
            start = start->path_child;
        }

        /* place */
        set_torso_height(0.3, true);
        set_left_arm_position_upper(0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0, true);
        set_right_arm_position_upper(0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0, true);
        // move the arms down
        set_left_arm_position_upper(0.0, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0, true);
        set_right_arm_position_upper(0.0, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0, true);
        //robot_go_forward(0.5);

        set_gripper(true, true, 20.0, true);
        set_gripper(false, true, 20.0, true);

        set_left_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, true);
        set_right_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, true);

        go_back(end_z,end_x,start_z,start_x,end_angle,start_angle+M_PI);

        break;
    }
}

/******************************** Main Function *********************************/

int main(int argc, char **argv) {
    wb_robot_init();

    initialize_devices();
    enable_devices();

    // get type&id by speech recognition
    // here's the code
    order_type=DOUBLE_BEER;
    guest_id=7;
    
    /******************iat_record begin*********************/
    
    int ret = MSP_SUCCESS;
    int upload_on = 0; /* whether upload the user word */
    /* login params, please do keep the appid correct */
    const char* login_params = "appid = 5a334825, work_dir = .";
    int aud_src = 1; /* from mic or file */

    /*
    * See "iFlytek MSC Reference Manual"
    */
    const char* session_begin_params =
      "sub = iat, domain = iat, language = zh_cn, "
      "accent = mandarin, sample_rate = 16000, "
      "result_type = plain, result_encoding = utf8";

    /* Login first. the 1st arg is username, the 2nd arg is password
     * just set them as NULL. the 3rd arg is login paramertes 
     * */
    ret = MSPLogin(NULL, NULL, login_params);
    if (MSP_SUCCESS != ret) {
      printf("MSPLogin failed , Error code %d.\n",ret);
      goto exit; // login fail, exit the program
    }

    // printf("Want to upload the user words ? \n0: No.\n1: Yes\n");
    // scanf("%d", &upload_on);
    upload_on = 0;

    if (upload_on)
    {
      printf("Uploading the user words ...\n");
      ret = upload_userwords();
      if (MSP_SUCCESS != ret)
        goto exit;  
      printf("Uploaded successfully\n");
    }

    // printf("Where the audio comes from?\n"
    //      "0: From a audio file.\n1: From microphone.\n");
    // scanf("%d", &aud_src);
    aud_src = 1;

    if(aud_src != 0) {
      printf("Demo recognizing the speech from microphone\n");
      printf("Speak in 15 seconds\n");

      demo_mic(session_begin_params);

      printf("15 sec passed\n");
    } else {
      printf("Demo recgonizing the speech from a recorded audio file\n");
      demo_file("wav/iflytek02.wav", session_begin_params); 
    }
  exit:
      MSPLogout(); // Logout...
    
    /******************iat_record end*********************/

    order_type = getfoodID(g_result);
    guest_id = getclientID(g_result);
    printf("order_type = %d    guest_id = %d\n", order_type, guest_id);
    
    set_initial_position(order_type);//torso height
    
    set_start_end_state(order_type, guest_id);

    serve(order_type, guest_id);

    wb_robot_cleanup();

    return EXIT_SUCCESS;
}

