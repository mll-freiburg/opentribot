
/*

    camera shared library

    Arjen den Hamer, 2005 - 2006
    reviewed by Rene van de Molengraft, 17 feb 2006
    last updated: March, 31th, 2006: added frame rate parameter to camera init

*/

#include "ProsilicaLib.h"

int init_camera(raw1394handle_t* ph, dc1394_cameracapture* pc, float frame_rate)
{
    raw1394handle_t handle;
    int numNodes;
    int numCameras;
    nodeid_t * camera_nodes;
    unsigned int colorcode;
    unsigned int framerate;
    float value;
    dc1394_feature_set features;
    unsigned int min_bytes,max_bytes;
    unsigned int actual_bytes;
    unsigned long long int total_bytes = 0;
    int i;

    int number_of_firewire_devices;
    struct raw1394_portinfo  portdata[3];

   /*--------------------------------------------------------------------
    *   Determine the number of fire-wire devices:
    *   If number_of_firewire_devices = 1 -> use 4-pin fire-wire in from of laptop
    *   If number_of_firewire_devices = 2 -> use 6-pin fire-wire in pcmcia socket
    *---------------------------------------------------------------------*/	

    handle = raw1394_new_handle();
    if (!handle) {
        printf("Unable to acquire raw1394 handle in init_camera() (Did you use 'sudo' ?)\n");
        return CAM_ERROR;
    }

    number_of_firewire_devices =  raw1394_get_port_info(handle, portdata,2);

    printf("number of firewire devices equals %i \n",number_of_firewire_devices);
    printf("Fire-wire device used: %s \n",portdata[number_of_firewire_devices-1].name);

    /* important, handle has to be destroyed in order to create handle with "dc1394_create_handle" (see dc1394.h) */
    raw1394_destroy_handle(handle);


    
  /*-----------------------------------------------------------------------
   *  Open ohci and asign handle to it
   *-----------------------------------------------------------------------*/
    handle=dc1394_create_handle(number_of_firewire_devices-1);
    if (handle==NULL)
    {
	fprintf(stderr,"Unable to aquire a raw1394 handle\n"
			"did you insmod the drivers?\n");
	return CAM_ERROR;
    }
/*  copy handle to user argument */
    *ph=handle;

  /*-----------------------------------------------------------------------
   *  get the camera nodes and describe them as we find them
   *-----------------------------------------------------------------------*/
    numNodes=raw1394_get_nodecount(handle);
    camera_nodes=dc1394_get_camera_nodes(handle,&numCameras,1);
    fflush(stdout);
    if (numCameras<1)
    {
	fprintf(stderr,"No cameras found :(\n");
	dc1394_destroy_handle(handle);
	return CAM_ERROR;
    }
    printf("Working with the first camera on the bus.\n");
    
  /*-----------------------------------------------------------------------
   *  to prevent the iso-transfer bug from raw1394 system, check if
   *  camera is highest node. For details see 
   *  http://linux1394.sourceforge.net/faq.html#DCbusmgmt
   *  and
   *  http://sourceforge.net/tracker/index.php?func=detail&aid=435107&group_id=8157&atid=108157
   *-----------------------------------------------------------------------*/

printf("test ");

    if (camera_nodes[0] == numNodes-1)
    {
	fprintf(stderr,"\n"
    	     "Sorry, your camera is the highest numbered node\n"
             "of the bus, and has therefore become the root node.\n"
             "The root node is responsible for maintaining \n"
             "the timing of isochronous transactions on the IEEE \n"
             "1394 bus.  However, if the root node is not cycle master \n"
             "capable (it doesn't have to be), then isochronous \n"
             "transactions will not work.  The host controller card is \n"
             "cycle master capable, however, most cameras are not.\n"
             "\n"
             "The quick solution is to add the parameter \n"
             "attempt_root=1 when loading the OHCI driver as a \n"
             "module.  So please do (as root):\n"
             "\n"
             "   rmmod ohci1394\n"
             "   insmod ohci1394 attempt_root=1\n"
             "\n"
             "for more information see the FAQ at \n"
             "http://linux1394.sourceforge.net/faq.html#DCbusmgmt\n"
             "\n");
	dc1394_destroy_handle(handle);
	return CAM_ERROR;
    }

   /* change mode */
   /*uint temp_mode;
   dc1394_get_operation_mode(handle,camera_nodes[0],&temp_mode);
   printf("mode: %i \n",temp_mode);
   dc1394_set_operation_mode(handle,camera_nodes[0],481);
   dc1394_get_operation_mode(handle,camera_nodes[0],&temp_mode);
   printf("mode: %i ******************************************* \n",temp_mode);*/


printf ("before get camera featuresset\n ");


    /*dc1394_set_brightness(handle,camera_nodes[0],255);*/
    /*dc1394_set_gamma(handle,camera_nodes[0],1);*/

    dc1394_get_camera_feature_set(handle,camera_nodes[0],&features);
    dc1394_print_feature_set(&features);  
   
/*  color coding */
    dc1394_query_format7_color_coding_id(handle,camera_nodes[0],MODE_FORMAT7_0,&colorcode); 
    printf("Current color code: %i\n",colorcode);
  
    dc1394_set_format7_color_coding_id(handle,camera_nodes[0],MODE_FORMAT7_0,COLOR_FORMAT7_YUV422);
    dc1394_query_format7_color_coding_id(handle,camera_nodes[0],MODE_FORMAT7_0,&colorcode);
    printf("New color code: %i\n ",colorcode);


/* get max of frame packetsize */

   uint min_packetsize,max_packetsize;
   dc1394_query_format7_packet_para(handle,camera_nodes[0],MODE_FORMAT7_0,&min_packetsize,&max_packetsize);
   printf("Max packetsize: %i \n",max_packetsize);    



/*  frame rate */ 

    dc1394_query_absolute_feature_value(handle,camera_nodes[0],FEATURE_FRAME_RATE,&value);
    printf("Absolute frame rate: %f \n",value);
    dc1394_set_absolute_feature_value(handle,camera_nodes[0],FEATURE_FRAME_RATE,frame_rate);

  /*-----------------------------------------------------------------------
   *  setup capture for format 7
   *-----------------------------------------------------------------------*/
    if (number_of_firewire_devices == 1)   /* use 4-pin fire-wire of laptop */
    {
        if( dc1394_dma_setup_format7_capture(handle,camera_nodes[0],
                                    0,
                                    MODE_FORMAT7_0,
                                    SPEED_400,
                                    4088,
                                    0,0,
                                    CAM_WIDTH,CAM_HEIGHT,
                                    5, 
                                    1,
                                    "/dev/video1394/0",
                                    pc) != DC1394_SUCCESS)
        {
            fprintf(stderr,"Unable to setup camera in format 7 mode 0-\n"
                "check line %d of %s to make sure\n"
                "that the video mode,framerate and format are\n"
                "supported by your camera\n",
                __LINE__,__FILE__);
            dc1394_destroy_handle(handle);
            return CAM_ERROR;
        }
    }
    else if (number_of_firewire_devices == 2)  /* use 6-pin fire-wire of pcmcia */
    {
        if( dc1394_dma_setup_format7_capture(handle,camera_nodes[0],
                                    0,
                                    MODE_FORMAT7_0,
                                    SPEED_400,
                                    4088,
                                    0,0,
                                    CAM_WIDTH,CAM_HEIGHT,
                                    1,
                                    1,
                                    "/dev/video1394_pcmcia",
                                    pc) != DC1394_SUCCESS)
        {
            fprintf(stderr,"Unable to setup camera in format 7 mode 0-\n"
                "check line %d of %s to make sure\n"
                "that the video mode,framerate and format are\n"
                "supported by your camera\n",
                __LINE__,__FILE__);
            dc1394_destroy_handle(handle);
            return CAM_ERROR;
        }
    }
    else
    {
        dc1394_destroy_handle(handle);
        return CAM_ERROR;
    }






   /*-----------------------------------------------------------------------
   *  have the camera start sending us data
   *-----------------------------------------------------------------------*/
    if (dc1394_start_iso_transmission(handle,pc->node)!=DC1394_SUCCESS) 
    {
	fprintf(stderr, "Unable to start camera iso transmission.\n");
	dc1394_release_camera(handle,pc);
	dc1394_destroy_handle(handle);
	return CAM_ERROR;
    }
      
/*  get frame-rate: standaard off = maximum */
    dc1394_query_absolute_feature_value(handle,camera_nodes[0],FEATURE_FRAME_RATE,&value);
    printf("New absolute frame rate: %f\n ",value);
  
/*  set trigger mode */
    if (dc1394_set_trigger_mode(handle, pc->node, TRIGGER_MODE_0)!=DC1394_SUCCESS)
    {
	fprintf(stderr, "Unable to set camera trigger mode.\n");
	dc1394_release_camera(handle,pc);
	dc1394_destroy_handle(handle);
	return CAM_ERROR;
    }
  
  /*-----------------------------------------------------------------------
   *  print allowed and used packet size
   *-----------------------------------------------------------------------*/
    if (dc1394_query_format7_packet_para(handle, camera_nodes[0], MODE_FORMAT7_0, &min_bytes, &max_bytes) != DC1394_SUCCESS) /* PACKET_PARA_INQ */
    {
	printf("dc1394_query_format7_packet_para error.\n");
    } else {
	printf( "Camera reports allowed packet size from %d - %d bytes.\n",min_bytes,max_bytes);
    }
    
    if (dc1394_query_format7_byte_per_packet(handle, camera_nodes[0], MODE_FORMAT7_0, &actual_bytes) != DC1394_SUCCESS)
    {
	printf("dc1394_query_format7_byte_per_packet error.\n");
    } else {
	printf("Camera reports actual packet size = %d bytes.\n",actual_bytes);
    }
    
/*  test single capture */
    if (dc1394_dma_single_capture(pc)!=DC1394_SUCCESS) 
    {    
	printf("Init: single capture not ok.\n");
	dc1394_release_camera(handle,pc);
	dc1394_destroy_handle(handle);
	return CAM_ERROR;
    }
    dc1394_dma_done_with_buffer(pc);
   
    printf("Initialization finished succesfully!\n");        

    return CAM_SUCCESS;  
}





int grab_image_YUV(unsigned char* pY, unsigned char* pU, unsigned char* pV, dc1394_cameracapture* pc)
{
    int i,j;

/*  grab new image */
    if (dc1394_dma_single_capture(pc)!=DC1394_SUCCESS) 
    {    
	printf("Single capture not ok.\n");
	return CAM_ERROR;
    }
    
/*  copy image to YUV user space (not in real-time context) */
    i=0;
    for (j=0;j<CAM_WIDTH*CAM_HEIGHT;j+=2) {
	pU[j]=	(unsigned char) ((pc->capture_buffer[i] & 0xff));
	pY[j]=	(unsigned char) ((pc->capture_buffer[i] & 0xff00)>>8);
	pV[j]=	(unsigned char) ((pc->capture_buffer[i] & 0xff0000)>>16);
	pY[j+1]=(unsigned char) ((pc->capture_buffer[i] & 0xff000000)>>24);	
	pU[j+1]=pU[j];	
	pV[j+1]=pV[j];
	i++;
    }
    
/*  ready with image */
    dc1394_dma_done_with_buffer(pc);
    
    return CAM_SUCCESS;
}




int set_camera_param(raw1394handle_t* ph, dc1394_cameracapture* pc,int* pparlist,int verbose_mode)
{
    

    /* param vector  
     * parlist[0] = white_balance_U  [0,255]
     * parlist[1] = white_balance_V  [0,255]
     * parlist[2] = brightness       [0,255]
     * parlist[3] = shutter          [1,4095]
     * parlist[4] = gain             [0,48]
     * parlist[5] = gamma            [0,1] 
     *
     * verbose_mode = 1: show feature settings after each call of set_camera_param
     * verbose_mode = 0: do not show feature settings
     *
     */
    
  
    dc1394_feature_set features; 
    
    /******************* This makes sure that features are enambled   (0=off, nonzero=on) **************/
    
    if( dc1394_feature_on_off(*ph, pc->node, FEATURE_WHITE_BALANCE, 1)!=DC1394_SUCCESS)  
        {
            printf("Error setting White balance on \n");
            return CAM_ERROR;
        }
    if( dc1394_feature_on_off(*ph, pc->node, FEATURE_BRIGHTNESS, 1)!=DC1394_SUCCESS)  
        {
            printf("Error setting Brightness on \n");
            return CAM_ERROR;
        }
    if( dc1394_feature_on_off(*ph, pc->node, FEATURE_SHUTTER, 1)!=DC1394_SUCCESS)  
        {
            printf("Error setting Shutter on \n");
            return CAM_ERROR;
        }
    if( dc1394_feature_on_off(*ph, pc->node, FEATURE_GAIN, 1)!=DC1394_SUCCESS) 
        {
            printf("Error setting Gain on \n");
            return CAM_ERROR;
        }
    if( dc1394_feature_on_off(*ph, pc->node, FEATURE_GAMMA, 1)!=DC1394_SUCCESS) 
        {
            printf("Error setting Gamma on \n");
            return CAM_ERROR;
        } 
                                          
    /******************** This sets the values of the features *******************************/
    if(dc1394_set_white_balance(*ph, pc->node, pparlist[0],  pparlist[1])!=DC1394_SUCCESS)
        {
            printf("Error setting white-balance parameter \n");
            return CAM_ERROR;
        }
     
    if(dc1394_set_feature_value(*ph, pc->node, FEATURE_BRIGHTNESS, pparlist[2])!=DC1394_SUCCESS)
        {
            printf("Error setting Brightness parameter \n");
            return CAM_ERROR;
        }
    
    if(dc1394_set_feature_value(*ph, pc->node, FEATURE_SHUTTER, pparlist[3])!=DC1394_SUCCESS)
        {
            printf("Error setting Shutter parameter \n");
            return CAM_ERROR;
        }
    
    if(dc1394_set_feature_value(*ph, pc->node, FEATURE_GAIN, pparlist[4])!=DC1394_SUCCESS)
        {
            printf("Error setting Gain parameter \n");
            return CAM_ERROR;
        }
    
    if(dc1394_set_feature_value(*ph, pc->node, FEATURE_GAMMA, pparlist[5])!=DC1394_SUCCESS)
        {
            printf("Error setting Gamma parameter \n");
            return CAM_ERROR;
        }
    
    if(verbose_mode != 0)
    {
        dc1394_get_camera_feature_set(*ph, pc->node,&features);
        dc1394_print_feature_set(&features); 
    }
    
    return CAM_SUCCESS;
    
}
    
    



int stop_camera(raw1394handle_t h, dc1394_cameracapture* pc)
{
    /*
    
    if(dc1394_dma_release_camera(h,pc)!=DC1394_SUCCESS)
        {
         printf("Error releasing camera \n");
         return CAM_ERROR;
        }
    */
    
    
    printf("destroying handle");

    dc1394_release_camera(h,pc);

    if(dc1394_destroy_handle(h)!=DC1394_SUCCESS)
        {
         printf("Error destroying handle camera \n");
         return CAM_ERROR;
        }
    
    return CAM_SUCCESS;
}


   


