#include<iostream>
#include<iomanip>
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//   library for Windows socket
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
// Thread for image proccessing
#include <pthread.h>   
#include <time.h>
using namespace std;

#define MAX_CLIENT 20
#define andport 22041
#define droneport 22042
const double pi = 3.1415926535897932384626433832795028841971693993751;

double completed[10], cost = 0;
int user;   // service requesting user number (target point number)
int* course; // global variable about TSP Path
int coursenum = 0;   // location of course array

struct Target {      // Latitude and Longitude per each user(user numbering)
   const char* pointNum;   // user number requesting while 3 min(target point number)
   double latitude;
   double longitude;
};

Target targetdata[20] = { {"0", 35.132974, 129.105759}, {"1", 35.133141, 129.105966 }, {"2", 35.133192, 129.106028 }, 
{"3", 35.133278, 129.106025 }, {"4", 35.132988, 129.106372 }, {"5", 35.134446, 129.1051705 }, {"6", 35.1344634, 129.1060115},
{"7", 35.1344634, 129.1060115 }, {"8", 35.134446, 129.1051705 }, {"9", 35.134446, 129.1051705 }, {"10", 35.134446, 129.1051705 },
{"11", 35.1344634, 129.1060115}, {"12", 35.1344634, 129.1060115 }, {"13", 35.134446, 129.1051705 }, {"14", 35.134446, 129.1051705 },
{"15", 35.134446, 129.1051705 }, {"16", 35.1344634, 129.1060115}, {"17", 35.1344634, 129.1060115 }, {"18", 35.134446, 129.1051705 },
{"19", 35.134446, 129.1051705 } };
// base point and allocated target point(Drone zone) GPS point  


string snddata;   // string to send Client

// to save user requesting GPS drone zone point information from Android app of User(client)
Target from_user[10];

// GPS point comparing function prototype
double toRad(double degree);      // degree to radian
double calculateDistance(double lat1, double long1, double lat2, double long2);   // to calculate GPS data

// TSP algorithm function Prototype
void takeInput(int villages, double** cordinates);
int least(int c, int villages, double** cordinates);
void mincost(int city, int villages, double** cordinates, int* course);

// Scoket connection and message function Prototype
string makemsg(string snddata, struct Target targetpoint[]);
void* t_function(void* data);      // for thraed to connect multiple Android user

int client_index = 0;      // current calling service user(Android app) number
int max_client = 0;

int main()
{
   /* service process
   get user wanting drone zone through Android app -> put requested drone zone data int TSP algorithm  
   -> calculating TSP path, and send shortest visiting path data to Drone client(Pixhawk-Pi) */

   int i, j;   // value for loop
   cout << fixed;
   cout.precision(6);
   
   clock_t begin, end;      // to count waiting time

   for(int i=0; i<10; i++){
      from_user[i].pointNum = {0};
      from_user[i].latitude = 0;
      from_user[i].longitude = 0;
   }

   // Variables for Socket
   int server_sock, client_sock;
   pthread_t thread_client[MAX_CLIENT];


   if ((server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)      // socket create
   {
      cout << "socket create error\n";
      return -1;
   }

   int on = 1;
   if (setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
   {
      cout << "socket option set error\n";
      return -1;
   }

   struct sockaddr_in server_addr, client_addr;
   int client_addr_size = sizeof(client_addr);
   server_addr.sin_addr.s_addr = INADDR_ANY;
   server_addr.sin_family = AF_INET;
   server_addr.sin_port = htons(andport);   // For android socket



   // Android socket start
   if (bind(server_sock, (struct sockaddr*) & server_addr, sizeof(server_addr)) < 0)      // bind
   {
      cout << "bind error\n";
      return -1;
   }

   if (listen(server_sock, 5) < 0)      // listen
   {
      cout << "listen error\n";
      return -1;
   }

   cout << "\nWaiting Client...\n\n";   // ready to connect Android client

   while (1)      // android socket connection
   {
   //   begin = clock();        // waiting time count


      if(client_index == 9)
         break;
      else client_sock = accept(server_sock, (struct sockaddr*) & client_addr, (socklen_t*)& client_addr_size);
      
      cout << "*****Client " << max_client + 1 << " information*****\nAddr : " << inet_ntoa(client_addr.sin_addr) << 
      "\nPort : " << ntohs(client_addr.sin_port) << "\n";

      if (client_sock < 0)
      {
         cout << "accept error\n";
      }

      if (max_client == MAX_CLIENT)
      {
         cout << "client accept full(max client count : " << MAX_CLIENT << "\n";
         close(client_sock);
         continue;
      }

      if (pthread_create(&thread_client[max_client], NULL, t_function, (void*)& client_sock) != 0)
      {
         cout << "Thread create error\n";
         close(client_sock);
         continue;
      }
      max_client++;   // max limitted client
   }
   cout << "Press Enter to calculate TSP algorithm\n";
   while (cin.get() != '\n');
   close(server_sock);
   // Android client socket connection finish
   cout << "\nAndroid service Disconnect!! Bye~!\n";



   // TSP algorithm
   user = client_index;   // copy current requesting service client number data to user for TSP algorithm 
   sleep(1000);

   Target* targetpoint = new Target; // target point to save from comparing targetdata and from_user data  
   targetpoint = (struct Target*)malloc(sizeof(struct Target) * user);      // struct malloc

   for (i = 0; i <= user; i++) {
      targetpoint[i].pointNum = targetdata[i].pointNum;
      targetpoint[i].latitude = targetdata[i].latitude;
      targetpoint[i].longitude = targetdata[i].longitude;
   }

   double** dist;      // To save distance calculating result from Android Client
   dist = new double* [user];   // 2dimension array dynamic memory allocation
   for (i = 0; i < user; i++){
      dist[i] = new double[user];
   }

   for (i = 0; i < user; i++) {      // make 2dimension array by using calculateDistance function
      for (j = 0; j < user; j++) {
         if (i == j) dist[i][j] = 0;
         else {
            dist[i][j] = calculateDistance(targetpoint[i].latitude, 
            targetpoint[i].longitude, targetpoint[j].latitude, targetpoint[j].longitude);
         }
      }
   }

   course = new int[user + 1];      // course dynamic memory allocation
   takeInput(user, dist);

   cout << "\n\nThe Path is\n=>";
   mincost(0, user, dist, course); //passing 0 because starting vertex

   cout << "\n\nMinimum cost is " << cost;

   snddata = makemsg(snddata, targetpoint);
   const char* msg = snddata.c_str();      // convert snddata to char type message
   // TSP algorithm finish and making path and message done



   // Drone socket connection start
   server_addr.sin_port = htons(droneport);   // For Drone(Pixhawk-Pi) socket

   cout << "\nWaiting Drone(Pi) Client...\n";      // raedy to connect Drone socket

   if (bind(server_sock, (struct sockaddr*) & server_addr, sizeof(server_addr)) < 0)      // bind
   {
      cout << "bind error\n";
      return -1;
   }

   if (listen(server_sock, 5) < 0)      // listen
   {
      cout << "listen error\n";
      return -1;
   }

   client_sock = accept(server_sock, (struct sockaddr*) & client_addr, (socklen_t*)& client_addr_size);
   if (client_sock < 0)
   {
      cout << "accept error\n";
   }

   if (write(client_sock, msg, BUFSIZ) <= 0)
   {
      printf("Client %d close\n", client_sock);
      close(client_sock);
   }

   cout << "Send Message : " << msg << "\n\n";
   
   char point[BUFSIZ] = "\0";      // message(visiting point or return back to base point(arrive)) from Client(Pixhawk-Pi)

   // get visiting target number information from Client
   cout << "Message Receives ...\n";
   while (1) {
      write(client_sock, point, BUFSIZ);

      if (strcmp(point, "arrive") == 0) {      // finish Socket when Server get arrive message
         printf("Close Clinet Connection..\n");
         break;
      }

      cout << "\nDrone Mission Complete\n";
   }

   close(client_sock);
   close(server_sock);
   // end of Socket connection
   cout << "\nDrone Disconnect!! Bye~!\n";
   // Drone client socket connection finish

   return 0;
}

double toRad(double degree) {
   return degree / 180 * pi;
}

// lat = Latitude, long = longitude
double calculateDistance(double lat1, double long1, double lat2, double long2) {
   double dist;
   dist = sin(toRad(lat1)) * sin(toRad(lat2)) + cos(toRad(lat1)) * cos(toRad(lat2)) * cos(toRad(long1 - long2));
   dist = acos(dist);
   //   dist = (6371 * pi * dist) / 180;
   //   got dist in radian, no need to change back to degree and convert to rad again.
   dist = 6371 * dist;
   return dist;
}

// villages : service requesting user number, cordinates : TSP 2dimension array by using point
void takeInput(int villages, double** cordinates)
 {
   int i, j;
   int n = villages;
   double** ary = cordinates;

   cout << "the number of villages : " << villages;

   cout << "\nTSP Cost Matrix\n";
   for (i = 0; i < n; i++) {
      for (j = 0; j < n; j++) {
         cout << ary[i][j] << "\t";
      }
      cout << "\n";

      completed[i] = 0;
   }

}

int least(int c, int villages, double** cordinates)
{
   int i, nc = 999;
   int min = 999;
   double kmin;
   double** ary = cordinates;
   int n = villages;

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

void mincost(int city, int villages, double** cordinates, int* course)
{
   int i, ncity;
   double** ary = cordinates;

   completed[city] = 1;

   cout << city << "--->";

   course[coursenum] = city;
   coursenum++;

   ncity = least(city, villages, ary);

   if (ncity == 999)
   {
      ncity = 0;
      cout << ncity + 1;
      cost += ary[city][ncity];

      return;
   }

   mincost(ncity, villages, ary, course);
}

string makemsg(string snddata, struct Target targetpoint[]) {
   int i, j;

   for (i = 0; i < user; i++) {
      for (j = 0; j < user; j++) {
         if (course[i] == atoi((targetpoint + j)->pointNum)) {
            snddata.append(to_string((targetpoint + j)->latitude));
            snddata.append("/");
            snddata.append(to_string((targetpoint + j)->longitude));
            snddata.append("/");
            break;
         }
      }
   }

   snddata.append(to_string((targetpoint + 0)->latitude));
   snddata.append("/");
   snddata.append(to_string((targetpoint + 0)->longitude));
   snddata.append("/");

   return snddata;
}

void* t_function(void* arg)
{
   sleep(1000);   //  wait for stablizing

   int client_sock = *((int*)arg);
   pid_t pid = getpid();      // process id
   pthread_t tid = pthread_self();  // thread id

   cout << "\nAndroid User Conecction Successfully!!\n";
   cout << "pid : " << (unsigned int)pid << ", tid : " << (unsigned int)tid << "\n";

   char clientpoint[BUFSIZ] = "\0";
   int i=0;

   while (1)
   {
      memset(clientpoint, 0x00, BUFSIZ);
      // compare and copy data to put and calculate TSP algorithm
      if (client_index < 9 && (read(client_sock, clientpoint, BUFSIZ) != 0)) {
         client_index++;   // add current service requesting client index
         i = max_client;   // to save client index while client thraed function run
         for (int cnt = 0; cnt < 20; cnt++) {
            if (strcmp(clientpoint, targetdata[cnt].pointNum) == 0) {   // comparing from_user and targetdata
               from_user[i].pointNum = targetdata[cnt].pointNum;
               from_user[i].latitude = targetdata[cnt].latitude;
               from_user[i].longitude = targetdata[cnt].longitude;
               char buf[512] = {"Booking complete! Thank you!"};
               (write(client_sock, buf, sizeof(buf)));
               break;
            }
         }
      }
      else{
         char buf[512] = {"Reservation is fulled! Plz Wait for next queue"};
         (write(client_sock, buf, sizeof(buf)));
         close(client_sock);
      }

      cout << "Client " << i << " ===>   Drone Zone : " << from_user[i].pointNum 
      << ",   Latitude : " << from_user[i].latitude << ",   Longitude : " << from_user[i].longitude << "\n";
      cout << "Client " << i << " close\n\n";
      close(client_sock);
      break;
   }
}