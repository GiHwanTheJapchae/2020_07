#include <iostream>
#include <iomanip>
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <WinSock2.h>    //   library for Windows socket
#include "SerialClass.h" // library for Arduino(using Serial.cpp/SerialClass.h)
#include <thread>        // Thread for image proccessing
using namespace std;
using std::thread;

#include <opencv2/opencv.hpp> // library for image processing
using namespace cv;

#define port 2204 // port num 2204

const double pi = 3.1415926535897932384626433832795028841971693993751;

double completed[10], cost = 0;
const int user = 8; // target point number
int *course;        // path array global variable
int coursenum = 0;  // array location of course

// Variables for Socket
// To connect scoket connection by using Wi-Fi(port 2204)
WSADATA wsaData;
SOCKET hServSock, hClntSock;
SOCKADDR_IN servAddr, clntAddr;
int szClntAddr;

string snddata; // string to send Client

struct TargetPt
{                // struct to send (user point number, Latitude, Longitude) to Client(Pixhawk-Pi)
    int usernum; // 각 좌표의 번호
    double x;    // 처음 입력되어 있는 각 지점의 좌표
    double y;
};

// prototype to calculate distance between each point
double toRad(double degree);                                                    // degree to radian
double calculateDistance(double lat1, double long1, double lat2, double long2); // to calculate GPS data

// prototype about TSP Algorithm
void takeInput(int points, double cordinates[user][user]);
double least(int c, int points, double cordinates[user][user]);
void mincost(int city, int points, double cordinates[user][user], int *course);

// prototype about socket
string makemsg(string snddata, struct TargetPt targetdata[user]);
void ErrorHandling(const char *message);

// prototype for thread
void socket_serial_thread();

int main(int argc, char *argv[])
{
    thread image_thread;

    TargetPt targetdata[user] = {{0, 35.134833, 129.106817}, {1, 35.134723, 129.105646}, {2, 35.134277, 129.105293}, {3, 35.134353, 129.105990}, {4, 35.134572, 129.106477}, {5, 35.134696, 129.105234}, {6, 35.134387, 129.106875}, {7, 35.134905, 129.106363}};
    // target point와 base포인트가 되는 좌표값

    int i, j;

    for (i = 0; i < user; i++)
    {
        cout << "Target[" << i + 1 << "] : (" << targetdata[i].x << ", " << targetdata[i].y << ")\n";
    }

    double dist[user][user]; // 받아온 좌표값들간 거리 계산 결과 저장 변수

    for (i = 0; i < user; i++)
    { // make 2d array by using calculateDistance
        for (j = 0; j < user; j++)
        {
            if (i == j)
                dist[i][j] = 0;
            else
            {
                dist[i][j] = calculateDistance(targetdata[i].x, targetdata[i].y,
                                               targetdata[j].x, targetdata[j].y);
            }
        }
    }

    course = new int[user + 1]; // course dynamic allocation

    // TSP 알고리즘
    takeInput(user, dist);

    cout << "\n\nThe Path is:\n";
    mincost(0, user, dist, course); //passing 0 because starting vertex

    cout << "\n\nMinimum cost is " << cost << "m\n";

    snddata = makemsg(snddata, targetdata);

    cout << "\nMessage : " << snddata << "\n"; // test

    thread t1(socket_serial_thread); // thread

    t1.join(); // wait until thread is finished

    return 0;
}
double toRad(double degree)
{
    return degree / 180 * pi;
}

// Indoor virture location(x, y) lat = Latitude(x), long = longitude(y)
double calculateDistance(double lat1, double long1, double lat2, double long2)
{
    double dist;
    dist = sin(toRad(lat1)) * sin(toRad(lat2)) + cos(toRad(lat1)) * cos(toRad(lat2)) * cos(toRad(long2 - long1));
    dist = acos(dist);
    //   dist = (6371 * pi * dist) / 180;
    //   got dist in radian, no need to change back to degree and convert to rad again.
    dist = 6371000 * dist;
    return dist;
}

void takeInput(int points, double cordinates[user][user]) // points=포인트 수, cordinates는 좌표를 이용한 TSP 2차원 배열
{
    int i, j;
    int n = points;

    cout << "\nthe number of points : " << points << "\n";

    cout << "\nTSP Cost Matrix\n";
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            cout.setf(ios::right);
            cout << setw(8) << cordinates[i][j] << "m\t";
        }
        cout << "\n";

        completed[i] = 0;
    }
}

double least(int c, int points, double cordinates[user][user])
{
    int i, nc = 999;
    int min = 999;
    double kmin;
    int n = points;

    for (i = 0; i < n; i++)
    {
        if ((cordinates[c][i] != 0) && (completed[i] == 0))
            if (cordinates[c][i] + cordinates[i][c] < min)
            {
                min = cordinates[i][c] + cordinates[c][i];
                kmin = cordinates[c][i];
                nc = i;
            }
    }

    if (min != 999)
        cost += kmin;

    return nc;
}

void mincost(int city, int points, double cordinates[user][user], int *course)
{
    int i, ncity;

    completed[city] = 1;

    cout << city + 1 << "--->";

    course[coursenum] = city;
    coursenum++;

    ncity = least(city, points, cordinates);

    if (ncity == 999)
    {
        ncity = 0;
        cout << ncity + 1;
        cost += cordinates[city][ncity];

        return;
    }

    mincost(ncity, points, cordinates, course);
}

string makemsg(string snddata, struct TargetPt targetdata[user])
{
    int i, j;

    for (i = 0; i < user; i++)
    {
        for (j = 0; j < user; j++)
        {
            if (course[i] == (targetdata + j)->usernum)
            {
                snddata.append(to_string((targetdata + j)->x));
                snddata.append("/");
                snddata.append(to_string((targetdata + j)->y));
                snddata.append("/");
                break;
            }
        }
    }

    snddata.append(to_string((targetdata + 0)->x));
    snddata.append("/");
    snddata.append(to_string((targetdata + 0)->y));
    snddata.append("/");

    return snddata;
}

void ErrorHandling(const char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

void socket_serial_thread()
{

    // To connect Ground pannel by USB Serial port
    printf("\n\nWaiting Pannel\n\n");
    Serial *SP = new Serial("\\\\.\\COM3");
    if (SP->IsConnected())
        cout << "Visiting Pannel connected\n\n";
    else
        cout << "Connect Failed\n\n";

    const char *msg = snddata.c_str(); // convert snddata to char type message
    char point[BUFSIZ] = "\0";         // message(visiting point or return back to base point(arrive)) from Client(Pixhawk-Pi)

    // Windows Socket initialization, before using Socket it should be initialized
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
        ErrorHandling("WSAStartup() error!");

    // make TCP Socket
    hServSock = socket(PF_INET, SOCK_STREAM, 0);

    if (hServSock == INVALID_SOCKET)
        ErrorHandling("socket() error!");

    // information about Socket
    memset(&servAddr, 0, sizeof(servAddr));

    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY); // using current PC IP address
    servAddr.sin_port = htons(port);              // port number 2204

    // 주소 지정
    if (::bind(hServSock, (SOCKADDR *)&servAddr, sizeof(servAddr)) == SOCKET_ERROR)
        ErrorHandling("bind() error!");

    // 접속 받을수 있게 함
    if (listen(hServSock, 5) == SOCKET_ERROR) // wating for connection during 5sec
        ErrorHandling("listen() error!");

    cout << "\n\nWaiting Client...\n";

    // Connection accept
    szClntAddr = sizeof(clntAddr);
    hClntSock = accept(hServSock, (SOCKADDR *)&clntAddr, &szClntAddr); // communication with Client

    if (hClntSock == INVALID_SOCKET)
        ErrorHandling("accept() error!");

    cout << "\nConecct Successfully!!\n";

    send(hClntSock, msg, BUFSIZ, 0);

    cout << "Send Message : " << msg << "\n\n";

    // get visiting target number information from Client
    cout << "Message Receives ...\n";
    while (1)
    {

        recv(hClntSock, point, BUFSIZ, 0);

        if (strcmp(point, "arrive") == 0)
        { // finish Socket when Server get arrive message
            printf("Close Clinet Connection..\n");
            break;
        }
        else
            cout << "Current Drone Point : " << point << "\n"; // about Arduino
        // 아두이노 지상 패널과 시리얼 통신 및 점등 코드 삽입
        SP->WriteData(point, 1);
    }

    cout << "\nDrone Mission Complete\n";
    closesocket(hClntSock);
    closesocket(hServSock);
    WSACleanup();
    // end of Socket connection
    cout << "\nDisconnect!! Bye~!\n";
}