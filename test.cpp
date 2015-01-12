/*
 * test.cpp
 *
 *  Created on: Nov 22, 2014
 *      Author: aylin
 */

#include"node.h"
#include"ImageProcess.h"




int main(int argc, char **argv)
{


   init(argc, argv, "Aylin_node");

    Subscribe_Depth sd;

    Rate spin_rate(10);

    while( ok() ) {
        spinOnce();

        spin_rate.sleep();
    }

   //depthSubtraction();



    return 0;
}


