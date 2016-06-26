
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <curses.h>
#include <iostream>

using namespace std;
double dUe1[500], dUf1[500], dVe1[500],dVf1[500];
double dUe2[500], dUf2[500], dVe2[500],dVf2[500];
double Ue1[500], Ue2[500];
double Uf1[500], Uf2[500];
double Ve1[500], Ve2[500];
double Vf1[500], Vf2[500];
double Ye1[500], Ye2[500];
double Yf1[500], Yf2[500];
double Y1[500];
double U0, b, step, T, t1, t2;


double Wfe;

int main()
{
    ofstream myfile ("output.txt");
    ofstream myfile2 ("output2.txt");
    int count = 0;
    step = 0.2;
    Wfe = 1.5; //lexor&Extensor weight connection
    t1 = 1;
    t2 = 12;
    U0 = 5;
    b = 2.5;

    Ue1[0] = 0.0;
    Ve1[0] = 0.0;

    Uf1[0] = 1.0;
    Vf1[0] = 1.0;

    while (count < 500)
    { 

        /*Extensor neuron 1*/

        dUe1[count] = (-Ue1[count] - (Wfe * Yf1[count]) - (b * Ve1[count]) + U0) / t1;
        Ue1[count + 1] = Ue1[count] + (step * dUe1[count]);
        Ye1[count + 1] = max(0.00, Ue1[count + 1]);

        dVe1[count] = (-Ve1[count] + Ye1[count + 1]) / t2;
        Ve1[count + 1] = Ve1[count] + (step * dVe1[count]);

        /*Flexor neuron 1*/

        dUf1[count] = (-Uf1[count] - (Wfe * Ye1[count]) - (b * Vf1[count]) + U0) / t1;
        Uf1[count + 1] = Uf1[count] + (step * dUf1[count]);
        Yf1[count + 1] = max(0.00, Uf1[count + 1]);

        dVf1[count] = (-Vf1[count] + Yf1[count + 1]) / t2;
        Vf1[count + 1] = Vf1[count] + (step * dVf1[count]);

        count++;
    } 

    for (int i=0; i<count; i++)
    {
    myfile << i*step<<" "<< Ye1[i] <<" "<<Yf1[i] <<  endl;
    myfile2 << i*step<<" "<< (Ye1[i] - Yf1[i]) <<  endl;
    }





}

