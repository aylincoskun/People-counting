/*
 * node.h
 *
 *  Created on: Nov 22, 2014
 *      Author: aylin
 */

#ifndef SRC_NODE_H_
#define SRC_NODE_H_

#include"waterfilling.h"
#include"ImageProcess.h"

class Subscribe_Depth {
	cv_bridge::CvImagePtr cv_ptr;
    NodeHandle nh;
    Subscriber sub;
    Mat a,b;
    double min,max;
    Ptr<BackgroundSubtractor> pMOG1;
    Mat fgMaskMOG1;



public:
    Subscribe_Depth() {

        sub = nh.subscribe("/camera/depth/image", 1, &Subscribe_Depth::callback, this);
        pMOG1 = new BackgroundSubtractorMOG(); //MOG2 approach



    }
Mat depthSubtrac(Mat depth_map, Mat back)
{
	typedef uchar imgType;
	    	   		imgType* m_a = NULL;
	    	   		imgType* m_b = NULL;
	    	   		imgType* m_c = NULL;

	    	   		int picWidth = depth_map.cols; //picture width
	    	   		int picHeight = depth_map.rows; //picture height

	    	   		Mat outputImg(picHeight,picWidth, depth_map.type()); //initialize the output image


	    	   		m_a=(imgType*)back.data;
	    	   		m_b=(imgType*)depth_map.data;
	    	   		m_c=(imgType*)outputImg.data;


	    	   		for(int i=0;i<picHeight*picWidth;i++,m_a++,m_b++,m_c++)
	    	   				{
	    	   					if(*m_a<2)
	    	   					{
	    	   						*m_a=0;
	    	   						*m_c = (*m_a)*(*m_b);


	    	   					}
	    	   					else
	    	   					{
	    	   						*m_a=1;
	    	   						*m_c = (*m_a)*(*m_b);


	    	   					}



	    	   				}

	    	   		return outputImg;
}

    ~Subscribe_Depth() {
     }

    void callback( const sensor_msgs::ImageConstPtr& im )
    {
    	Mat depth_map, sub_map, filter_map,new_im,temp;


    	  //convert ros to opencv (32 bits float)
    	  cv_ptr = cv_bridge::toCvCopy(im, sensor_msgs::image_encodings::TYPE_32FC1);


    	   //min max range
    	   minMaxIdx(cv_ptr->image, &min, &max);

    	   // expand your range to 0..255. Similar to histEq();
    	   cv_ptr->image.convertTo(depth_map,CV_8U, 255 / (max-min), -min);

    	   pMOG1->operator()(depth_map, fgMaskMOG1);




    	 Mat  outputImg= depthSubtrac(depth_map,fgMaskMOG1);

    	 WaterFilling water_obje;

    	 Mat result=water_obje.WaterDrop(outputImg,1000);


    	   		//imshow("depth",depth_map);
    	   		//imshow("out",outputImg);
    	   		imshow("waterfilling",result);
    	   		waitKey(3);


  }



};

#endif /* SRC_NODE_H_ */
