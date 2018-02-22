
#include "mipi_dsih_dphy.h" 

#define PRECISION_FACTOR        (1000)                                                                                                                                 
                                                                                       
#define DPHY_DIV_UPPER_LIMIT    (40000)                                                                                                                                
#ifdef GEN_2
#define DPHY_DIV_LOWER_LIMIT	(5000)
#else
#define DPHY_DIV_LOWER_LIMIT    (1000)                                                                                                                                 
#endif
#if ((defined DWC_MIPI_DPHY_BIDIR_TSMC40LP) || (defined GEN_2))
#define MIN_OUTPUT_FREQ         (80)                                                                                                                                   
#elif defined DPHY2Btql                                                                                                                                                
#define MIN_OUTPUT_FREQ         (200)                                                                                                                                  
#undef GEN_2
#endif                                                                                                                                                                 
dsih_error_t mipi_dsih_dphy_open(dphy_t * phy)                                                                                                                         
{                                                                                                                                                                      
    if (phy == 0)                                                                                                                                                      
    {                                                                                                                                                                  
        return ERR_DSI_PHY_INVALID;                                                                                                                                    
    }                                                                                                                                                                  
    else if ((phy->core_read_function == 0) || (phy->core_write_function == 0))                                                                                        
    {                                                                                                                                                                  
        return ERR_DSI_INVALID_IO;                                                                                                                                     
    }                                                                                                                                                                  
    else if (phy->status == INITIALIZED)                                                                                                                               
    {                                                                                                                                                                  
        return ERR_DSI_PHY_INVALID;                                                                                                                                    
    }                                                                                                                                                                  
    phy->status = NOT_INITIALIZED;
#if 0
    mipi_dsih_dphy_reset(phy, 0);                                                                                                                                      
    mipi_dsih_dphy_stop_wait_time(phy, 0x1C);                                                                                                                          
    mipi_dsih_dphy_no_of_lanes(phy, 1);                                                                                                                                
    mipi_dsih_dphy_clock_en(phy, 1);                                                                                                                                   
    mipi_dsih_dphy_shutdown(phy, 1);                                                                                                                                   
    mipi_dsih_dphy_reset(phy, 1);
#endif
    phy->status = INITIALIZED;                                                                                                                                         
    return OK;                                                                                                                                                         
}                                                                                                                                                                      
dsih_error_t mipi_dsih_dphy_configure(dphy_t * phy, uint8_t no_of_lanes, uint32_t output_freq)                                                                         
{                                                                                                                                                                      
    uint32_t loop_divider = 0;                                                                                                                                
    uint32_t input_divider = 1;                                                                                                                               
    uint8_t data[4];                                                                                                
    uint8_t no_of_bytes = 0;                                                                                                                                           
    uint8_t i = 0;
    uint8_t n=0;
    uint8_t range = 0;                                                                                                                            
    int flag = 0;                                                                                                                                                      
	struct
	{
		uint32_t loop_div; 
		uint8_t cp_current; 
		uint8_t lpf_resistor; 
	}
	loop_bandwidth[] =

#ifdef GEN_2
	{	
		{  90, 0x02, 0x02}, { 100, 0x02, 0x02}, { 110, 0x02, 0x02},
		{ 130, 0x02, 0x01}, { 140, 0x02, 0x01}, { 150, 0x02, 0x01},
		{ 170, 0x09, 0x00}, { 180, 0x09, 0x01}, { 200, 0x09, 0x01},
		{ 220, 0x09, 0x04}, { 240, 0x09, 0x04}, { 250, 0x09, 0x04},
		{ 270, 0x06, 0x04}, { 300, 0x06, 0x04}, { 330, 0x09, 0x04},
		{ 360, 0x09, 0x04}, { 400, 0x09, 0x04}, { 450, 0x06, 0x04},
		{ 500, 0x06, 0x04}, { 550, 0x06, 0x04}, { 600, 0x06, 0x04},
		{ 650, 0x0A, 0x04}, { 700, 0x0A, 0x04}, { 750, 0x0A, 0x04},
		{ 800, 0x0A, 0x04}, { 850, 0x0A, 0x04}, { 900, 0x0A, 0x04},
		{ 950, 0x0B, 0x08}, {1000, 0x0B, 0x08}, {1050, 0x0B, 0x08},
		{1100, 0x0B, 0x08}, {1150, 0x0B, 0x08}, {1200, 0x0B, 0x08},
		{1250, 0x0B, 0x08}, {1300, 0x0B, 0x08}, {1350, 0x0B, 0x08},
		{1400, 0x0B, 0x08}, {1450, 0x0B, 0x08}, {1500, 0x0B, 0x08}
	};
	uint32_t delta = 0;
	uint32_t tmp_loop_divider = 0;
	unsigned step = 0;
#endif
#ifdef DWC_MIPI_DPHY_BIDIR_TSMC40LP                                                                                                                                    
	{
		{32, 0x06, 0x10}, {64, 0x06, 0x10}, {128, 0x0C, 0x08},
		{256, 0x04, 0x04}, {512, 0x00, 0x01}, {768, 0x01, 0x01},
		{1000, 0x02, 0x01}
	};
#endif
#if ((defined DWC_MIPI_DPHY_BIDIR_TSMC40LP) || (defined GEN_2))
    struct                                                                                                                                                             
    {                                                                                                                                                                  
        uint32_t freq;                                                                                                            
        uint8_t hs_freq;                                                                                                                              
        uint8_t vco_range;                                                                                                                               
    }                                                                                                                                                                  
    ranges[] =                                                                                                                                                         
#ifdef GEN_2
	{
		{  90, 0x00, 0x00}, { 100, 0x10, 0x00}, { 110, 0x20, 0x00},
		{ 130, 0x01, 0x00}, { 140, 0x11, 0x00}, { 150, 0x21, 0x00},
		{ 170, 0x02, 0x00}, { 180, 0x12, 0x00}, { 200, 0x22, 0x00},
		{ 220, 0x03, 0x01}, { 240, 0x13, 0x01}, { 250, 0x23, 0x01},
		{ 270, 0x04, 0x01}, { 300, 0x14, 0x01}, { 330, 0x05, 0x02},
		{ 360, 0x15, 0x02}, { 400, 0x25, 0x02}, { 450, 0x06, 0x02},
		{ 500, 0x16, 0x02}, { 550, 0x07, 0x03}, { 600, 0x17, 0x03},
		{ 650, 0x08, 0x03}, { 700, 0x18, 0x03}, { 750, 0x09, 0x04},
		{ 800, 0x19, 0x04}, { 850, 0x29, 0x04}, { 900, 0x39, 0x04},
		{ 950, 0x0A, 0x05}, {1000, 0x1A, 0x05}, {1050, 0x2A, 0x05},
		{1100, 0x3A, 0x05}, {1150, 0x0B, 0x06}, {1200, 0x1B, 0x06},
		{1250, 0x2B, 0x06}, {1300, 0x3B, 0x06}, {1350, 0x0C, 0x07},
		{1400, 0x1C, 0x07}, {1450, 0x2C, 0x07}, {1500, 0x3C, 0x07}
	};
#endif
#ifdef DWC_MIPI_DPHY_BIDIR_TSMC40LP
    {                                                                                                                                                                  
        {90, 0x00, 0x01}, {100, 0x10, 0x01}, {110, 0x20, 0x01},                                                                                                        
        {125, 0x01, 0x01}, {140, 0x11, 0x01}, {150, 0x21, 0x01},                                                                                                       
        {160, 0x02, 0x01}, {180, 0x12, 0x03}, {200, 0x22, 0x03},                                                                                                       
        {210, 0x03, 0x03}, {240, 0x13, 0x03}, {250, 0x23, 0x03},                                                                                                       
        {270, 0x04, 0x07}, {300, 0x14, 0x07}, {330, 0x24, 0x07},                                                                                                       
        {360, 0x15, 0x07}, {400, 0x25, 0x07}, {450, 0x06, 0x07},                                                                                                       
        {500, 0x16, 0x07}, {550, 0x07, 0x0f}, {600, 0x17, 0x0f},                                                                                                       
        {650, 0x08, 0x0f}, {700, 0x18, 0x0f}, {750, 0x09, 0x0f},                                                                                                       
        {800, 0x19, 0x0f}, {850, 0x0A, 0x0f}, {900, 0x1A, 0x0f},                                                                                                       
        {950, 0x2A, 0x0f}, {1000, 0x3A, 0x0f}                                                                                                                          
    };                                                                                                                                                                 
#endif

#elif defined DPHY2Btql
    {
        {32, 0x0B, 0x00}, {64, 0x0A, 0x00}, {128, 0x09, 0x01},                                                                                                         
        {256, 0x08, 0x03}, {512, 0x08, 0x07}, {768, 0x08, 0x0F},                                                                                                       
        {1000, 0x08, 0x1F}                                                                                                                                             
    };                                                                                                                                                                 
	
	range = 1;
#endif                                                                                                                                                                 
    if (phy == 0)                                                                                                                                                      
    {                                                                                                                                                                  
        return ERR_DSI_INVALID_INSTANCE;                                                                                                                               
    }                                                                                                                                                                  
    if (phy->status < INITIALIZED)                                                                                                                                     
    {                                                                                                                                                                  
        return ERR_DSI_INVALID_INSTANCE;                                                                                                                               
    }                                                                                                                                                                  
    if (output_freq < MIN_OUTPUT_FREQ)                                                                                                                                 
    {                                                                                                                                                                  
        return ERR_DSI_PHY_FREQ_OUT_OF_BOUND;                                                                                                                          
    }                                                                                                                                                                  
#ifndef GEN_2
	
    for (input_divider = 1 + (phy->reference_freq / DPHY_DIV_UPPER_LIMIT); ((phy->reference_freq / input_divider) >= DPHY_DIV_LOWER_LIMIT) && (!flag); input_divider++)
    {                                                                         
        if (((output_freq * input_divider) % (phy->reference_freq )) == 0)                                                                                             
        {                                                                                                                                            
            loop_divider = ((output_freq * input_divider) / (phy->reference_freq ));                                                                                   
            if (loop_divider >= 12)                                                                                                                                    
            {                                                                                                                                                          
                flag = 1;                                                                                                                                              
				
				input_divider--;
            }                                                                                                                                                          
        }                                                                                                                                                              
    }                                                                                                                                                                  
    if ((!flag) || ((phy->reference_freq / input_divider) < DPHY_DIV_LOWER_LIMIT))                                                                                     
    {                                                                                                                   
                                                                                                       
        loop_divider = output_freq / DPHY_DIV_LOWER_LIMIT;                                                                                                             
        input_divider = phy->reference_freq / DPHY_DIV_LOWER_LIMIT;                                                                                                    
    }                                                                                                                                                                  
    else                                                                                                                                                               
#else
	loop_divider = ((output_freq * (phy->reference_freq / DPHY_DIV_LOWER_LIMIT)) / phy->reference_freq);
	
	delta = ((loop_divider * phy->reference_freq) / (phy->reference_freq / DPHY_DIV_LOWER_LIMIT)) - output_freq;
	for (input_divider = 1 + (phy->reference_freq / DPHY_DIV_UPPER_LIMIT); ((phy->reference_freq / input_divider) >= DPHY_DIV_LOWER_LIMIT) && (!flag); input_divider++)
	{
		tmp_loop_divider = ((output_freq * input_divider) / (phy->reference_freq));
		if ((tmp_loop_divider % 2) == 0)
		{	
			if (output_freq == (tmp_loop_divider * (phy->reference_freq / input_divider)))
			{	
				flag = 1;
				loop_divider = tmp_loop_divider;
				delta = output_freq - (tmp_loop_divider * (phy->reference_freq / input_divider));
				
				input_divider--;
			}
			if ((output_freq - (tmp_loop_divider * (phy->reference_freq / input_divider))) < delta)
			{	
				loop_divider = tmp_loop_divider;
				delta = output_freq - (tmp_loop_divider * (phy->reference_freq / input_divider));
				step = 1;
			}
		}
		else
		{
			tmp_loop_divider += 1;
			if (output_freq == (tmp_loop_divider * (phy->reference_freq / input_divider)))
			{	
				flag = 1;
				loop_divider = tmp_loop_divider;
				delta = (tmp_loop_divider * (phy->reference_freq / input_divider)) - output_freq;
				
				input_divider--;
			}
			if (((tmp_loop_divider * (phy->reference_freq / input_divider)) - output_freq) < delta)
			{	
				loop_divider = tmp_loop_divider;
				delta = (tmp_loop_divider * (phy->reference_freq / input_divider)) - output_freq;
				step = 0;
			}
		}
	}
	if (!flag)
	{
		input_divider = step + (loop_divider * phy->reference_freq) / output_freq;
	}
#endif
#ifdef DPHY2Btql
    for (i = 0; (i < (sizeof(loop_bandwidth)/sizeof(loop_bandwidth[0]))) && (loop_divider > loop_bandwidth[i].loop_div); i++)
    {                                                                                                                                                                  
        ;                                                                                                                                                              
    }                                                                                                                                                                  
    if (i >= (sizeof(loop_bandwidth)/sizeof(loop_bandwidth[0])))
    {                                                                                                                                                                  
        return ERR_DSI_PHY_FREQ_OUT_OF_BOUND;                                                                                                                          
    }                                                                                                                                                                  
                                                                                    
#endif
    mipi_dsih_dphy_reset(phy, 0);                                                                                                                                      
    mipi_dsih_dphy_clock_en(phy, 0);                                                                                                                                   
    mipi_dsih_dphy_shutdown(phy, 0);                                                                                                                                   
                                                                                                      
    mipi_dsih_dphy_test_clear(phy, 1);                                                                                                                                 
    mipi_dsih_dphy_test_clear(phy, 0);                                                                                                                                 
    for(n=0;n<100;n++){
            ;
    }
#if ((defined DWC_MIPI_DPHY_BIDIR_TSMC40LP) || (defined GEN_2))
                                                                                                                                                      
    for (range = 0; (range < (sizeof(ranges)/sizeof(ranges[0]))) && ((output_freq / 1000) > ranges[range].freq); range++)                                              
    {                                                                                                                                                                  
        ;                                                                                                                                                              
    }                                                                                                                                                                  
    if (range >= (sizeof(ranges)/sizeof(ranges[0])))                                                                                                                   
    {                                                                                                                                                                  
        return ERR_DSI_PHY_FREQ_OUT_OF_BOUND;                                                                                                                          
    }                                                                                                                                                                  
                                                                                                                     
    if (phy->bsp_pre_config != 0)                                                                                                                                      
    {                                                                                                                                                                  
        phy->bsp_pre_config(phy, 0);                                                                                                                                   
    }

#if defined(CONFIG_MACH_DUMMY)
{
    
    data[0] =  0x42;
    mipi_dsih_dphy_write(phy, 0x60, data, 1);


    data[0] =  0x42;
    mipi_dsih_dphy_write(phy, 0x70, data, 1);
}
#else
{
    data[0] =  0x42;
    mipi_dsih_dphy_write(phy, 0x60, data, 1);

	data[0] =  0x42;
    mipi_dsih_dphy_write(phy, 0x70, data, 1);
}
#endif




    
    

    
    

    

                                                                                                                                               
                                                                                                                                  
    data[0] = (0 << 7) | (ranges[range].hs_freq << 1) | 0;    
   
   
    mipi_dsih_dphy_write(phy, 0x44, data, 1);       
    data[1] = mipi_dsih_dphy_test_data_out(phy);
    printk("[DISP] mipi_dsih_dphy_configure-->0x44:%x \n",data[1]);
                                                                                                                                                        
                                                                                                                                   
    data[0] = (1 << 7) | (ranges[range].vco_range << 3) | (0 << 1) | 0;                                                                                                
    mipi_dsih_dphy_write(phy, 0x10, data, 1);                 
    data[1] = mipi_dsih_dphy_test_data_out(phy);
    printk("[DISP] mipi_dsih_dphy_configure-->0x10:%x \n",data[1]);
#ifdef GEN_2
#ifdef TESTCHIP
	
	data[0] = 0x80;
	mipi_dsih_dphy_write(phy, 0x32, data, 1);
	mipi_dsih_dphy_write(phy, 0x42, data, 1);
	mipi_dsih_dphy_write(phy, 0x52, data, 1);
	mipi_dsih_dphy_write(phy, 0x82, data, 1);
	mipi_dsih_dphy_write(phy, 0x92, data, 1);
#endif
	if ((loop_divider % 2) != 0)
	{	
		loop_divider -= 1;

	}
	i = range;
	data[0] = (0x00 << 6) | (0x01 << 5) | (0x01 << 4);
#endif
#ifdef DWC_MIPI_DPHY_BIDIR_TSMC40LP
                                                                  
    data[0] = (0x00 << 6) | (0x01 << 5) | (0x01 << 4) | (0x03 << 0);                                 
#endif
    mipi_dsih_dphy_write(phy, 0x19, data, 1);      
    data[1] = mipi_dsih_dphy_test_data_out(phy);
    printk("[DISP] mipi_dsih_dphy_configure-->0x19:%x \n",data[1]);
#elif defined DPHY2Btql                                                                                                                                                
                                                                                                                               
    data[0] =  ((((output_freq / 1000) > 500 )? 1: 0) << 4) | (1 << 3) | (0 << 1) | 0;                                                                                 
    mipi_dsih_dphy_write(phy, 0x10, data, 1);                                                                                                                          
#endif                                                                                                                                                                 
                                                                                                                  
    data[0] = (0x00 << 4) | (loop_bandwidth[i].cp_current << 0);
    mipi_dsih_dphy_write(phy, 0x11, data, 1);           
    data[1] = mipi_dsih_dphy_test_data_out(phy);
    printk("[DISP] mipi_dsih_dphy_configure-->0x11:%x \n",data[1]);
                                                                                                 
    data[0] = (0x01 << 7) | (0x01 << 6) |(loop_bandwidth[i].lpf_resistor << 0);
    mipi_dsih_dphy_write(phy, 0x12, data, 1);                                                                                                                          
    data[1] = mipi_dsih_dphy_test_data_out(phy);
    printk("[DISP] mipi_dsih_dphy_configure-->0x12:%x \n",data[1]);
                                                                                                                                    
   data[0] = input_divider - 1;                                                                                                                                       
   mipi_dsih_dphy_write(phy, 0x17, data, 1);           
    data[1] = mipi_dsih_dphy_test_data_out(phy);
    printk("[DISP] mipi_dsih_dphy_configure-->0x17:%x \n",data[1]);
    
   
    
		data[0]=0x8B;
		mipi_dsih_dphy_write(phy, 0x22, data, 1);

		data[1] = mipi_dsih_dphy_test_data_out(phy);
		printk("mipi_dsih_dphy 0x22 data[0]= %x,data[1] = %x \n",data[0],data[1]);

		data[0] =0x1E;
		mipi_dsih_dphy_write(phy, 0x21, data, 1);

		data[1] = mipi_dsih_dphy_test_data_out(phy);
		printk("mipi_dsih_dphy 0x21 data[0]= %x,data[1] = %x \n",data[0],data[1]);
    
	

    no_of_bytes = 2;                                                                            
    for (i = 0; i < no_of_bytes; i++)
    {                                                                                                                                                                  
        data[i] = ((uint8_t)((((loop_divider - 1) >> (5 * i)) & 0x1F) | (i << 7) ));
        /* 7 is dependent on no_of_bytes                                                                                                                               
        make sure 5 bits only of value are written at a time */                                                                                                        
    }                                                                                                                                                                  
    printk("[DISP] mipi_dsih_dphy_configure-->0x18:%x,:%x \n",data[0],data[1]);
                                                                                        
    mipi_dsih_dphy_write(phy, 0x18, data, no_of_bytes);                                                                                                                
    mipi_dsih_dphy_no_of_lanes(phy, no_of_lanes);                                                                                                                      
    mipi_dsih_dphy_stop_wait_time(phy, 0x1C);                                                                                                                          
    mipi_dsih_dphy_clock_en(phy, 1);                                                                                                                                   
    for(n=0;n<100;n++){
            ;
    }
    mipi_dsih_dphy_shutdown(phy, 1);                                                                                                                                   
    for(n=0;n<100;n++){
            ;
    }
    mipi_dsih_dphy_reset(phy, 1);                                                                                                                                      
    return OK;                                                                                                                                                         
} 

dsih_error_t mipi_dsih_dphy_close(dphy_t * phy)                                                                                                                        
{                                                                                                                                                                      
    if (phy == 0)                                                                                                                                                      
    {                                                                                                                                                                  
        return ERR_DSI_INVALID_INSTANCE;                                                                                                                               
    }                                                                                                                                                                  
    else if ((phy->core_read_function == 0) || (phy->core_write_function == 0))                                                                                        
    {                                                                                                                                                                  
        return ERR_DSI_INVALID_IO;                                                                                                                                     
    }                                                                                                                                                                  
    if (phy->status < NOT_INITIALIZED)                                                                                                                                 
    {                                                                                                                                                                  
        return ERR_DSI_INVALID_INSTANCE;                                                                                                                               
    }                                                                                                                                                                  
    mipi_dsih_dphy_reset(phy, 0);                                                                                                                                      
    mipi_dsih_dphy_reset(phy, 1);                                                                                                                                      
    mipi_dsih_dphy_shutdown(phy, 0);                                                                                                                                   
    phy->status = NOT_INITIALIZED;                                                                                                                                     
    return OK;                                                                                                                                                         
}                                                                                                                                                                      
void mipi_dsih_dphy_clock_en(dphy_t * instance, int en)                                                                                                                
{                                                                                                                                                                      
    mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_RSTZ, en, 2, 1);                                                                                                
}                                                                                                                                                                      
void mipi_dsih_dphy_reset(dphy_t * instance, int reset)                                                                                                                
{                                                                                                                                                                      
    mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_RSTZ, reset, 1, 1);                                                                                             
}                                                                                                                                                                      
void mipi_dsih_dphy_shutdown(dphy_t * instance, int powerup)                                                                                                           
{                                                                                                                                                                      
    mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_RSTZ, powerup, 0, 1);                                                                                           
}                                                                                                                                                                      
void mipi_dsih_dphy_stop_wait_time(dphy_t * instance, uint8_t no_of_byte_cycles)                                                                                       
{                                                                                                                                                                      
    mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CFG, no_of_byte_cycles, 2, 8);                                                                               
}                                                                                                                                                                      
void mipi_dsih_dphy_no_of_lanes(dphy_t * instance, uint8_t no_of_lanes)                                                                                                
{                                                                                                                                                                      
    mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CFG, no_of_lanes - 1, 0, 2);                                                                                 
}                                                                                                                                                                      
uint8_t mipi_dsih_dphy_get_no_of_lanes(dphy_t * instance)                                                                                                              
{                                                                                                                                                                      
    return mipi_dsih_dphy_read_part(instance, R_DSI_HOST_PHY_IF_CFG, 0, 2);                                                                                            
}                                                                                                                                                                      
void mipi_dsih_dphy_enable_hs_clk(dphy_t * instance, int enable)                                                                                                       
{                                                                                                                                                                      
    mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, enable, 0, 1);                                                                                         
}                                                                                                                                                                      
dsih_error_t mipi_dsih_dphy_escape_mode_trigger(dphy_t * instance, uint8_t trigger_request)                                                                            
{                                                                                                                                                                      
    uint8_t sum = 0;                                                                                                                                                   
    int i = 0;                                                                                                                                                         
    for (i = 0; i < 4; i++)                                                                                                                                            
    {                                                                                                                                                                  
        sum += ((trigger_request >> i) & 1);                                                                                                                           
    }                                                                                                                                                                  
    if (sum == 1)                                                                                                                                                      
    {                                                                                                                                           
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0x00, 5, 4);                                                                                       
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, trigger_request, 5, 4);                                                                            
        for (i = 0; i < DSIH_PHY_ACTIVE_WAIT; i++)                                                                                                                     
        {                                                                                                                                                              
            if(mipi_dsih_dphy_status(instance, 0x0010))                                                                                                                
            {                                                                                                                                                          
                break;                                                                                                                                                 
            }                                                                                                                                                          
        }                                                                                                                                                              
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0x00, 5, 4);                                                                                       
        if (i >= DSIH_PHY_ACTIVE_WAIT)                                                                                                                                 
        {                                                                                                                                                              
            return ERR_DSI_TIMEOUT;                                                                                                                                    
        }                                                                                                                                                              
        return OK;                                                                                                                                                     
    }                                                                                                                                                                  
    return ERR_DSI_INVALID_COMMAND;                                                                                                                                    
}                                                                                                                                                                      
#ifdef GEN_2
dsih_error_t mipi_dsih_dphy_ulps_data_lanes(dphy_t * instance, int enable)
{
    int timeout;
	
	uint16_t data_lanes_mask = 0;
    if (enable)
    {
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 1, 3, 1);
		return OK;
    }
    else
    {
		if (mipi_dsih_dphy_status(instance, 0x1) == 0)
		{
			return ERR_DSI_PHY_PLL_NOT_LOCKED;
		}
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 1, 4, 1);
		switch (mipi_dsih_dphy_get_no_of_lanes(instance))
		{
			case 3:
				data_lanes_mask |= (1 << 12);
			case 2:
				data_lanes_mask |= (1 << 10);
			case 1:
				data_lanes_mask |= (1 << 8);
			case 0:
				data_lanes_mask |= (1 << 5);
				break;
			default:
				data_lanes_mask = 0;
				break;
		}
        for (timeout = 0; timeout < DSIH_PHY_ACTIVE_WAIT; timeout++)
        {   
            
            if (mipi_dsih_dphy_status(instance, data_lanes_mask) == data_lanes_mask)
            {
				break;
			}
				
                for (timeout = 0; timeout < ONE_MS_ACTIVE_WAIT; timeout++)
                {
                    ;
                }
            }
		if (mipi_dsih_dphy_status(instance, data_lanes_mask) != data_lanes_mask)
		{
			instance->log_info("[DISP] stat %x, mask %x", mipi_dsih_dphy_status(instance, data_lanes_mask), data_lanes_mask);
			return ERR_DSI_TIMEOUT;
        }
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0, 3, 1);
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0, 4, 1);
    }
	return OK;
}
#else
void mipi_dsih_dphy_ulps_data_lanes(dphy_t * instance, int enable)                                                                                                     
{                                                                                                                                                                      
    int timeout;                                                                                                                                                       
    if (enable)                                                                                                                                                        
    {                                                                                                                                                                  
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 1, 3, 1);                                                                                          
    }                                                                                                                                                                  
    else                                                                                                                                                               
    {                                                                                                                                                                  
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 1, 4, 1);                                                                                          
        for (timeout = 0; timeout < DSIH_PHY_ACTIVE_WAIT; timeout++)                                                                                                   
        {                                                                                                                      
                                                                                                                                               
            if (mipi_dsih_dphy_status(instance, 0x1520) == 0)                                                                                                          
            {                                                                                                                                   
                for (timeout = 0; timeout < ONE_MS_ACTIVE_WAIT; timeout++)                                                                                             
                {                                                                                                                                                      
                    ;                                                                                                                                                  
                }                                                                                                                                                      
                break;                                                                                                                                                 
            }                                                                                                                                                          
        }                                                                                                                                                              
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0, 3, 1);                                                                                          
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0, 4, 1);                                                                                          
    }                                                                                                                                                                  
}
#endif
#ifdef GEN_2
dsih_error_t mipi_dsih_dphy_ulps_clk_lane(dphy_t * instance, int enable)
{
    int timeout;
	
	uint16_t clk_lane_mask = 0x0008;
    if (enable)
    {
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 1, 1, 1);
    }
    else
    {
		if (mipi_dsih_dphy_status(instance, 0x1) == 0)
		{
			return ERR_DSI_PHY_PLL_NOT_LOCKED;
		}
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 1, 2, 1);
        for (timeout = 0; timeout < DSIH_PHY_ACTIVE_WAIT; timeout++)
        {   
            
            if (mipi_dsih_dphy_status(instance, clk_lane_mask) == clk_lane_mask)
            {   
				instance->log_info("[DISP] stat %x, mask %x", mipi_dsih_dphy_status(instance, clk_lane_mask), clk_lane_mask);
				break;
			}
			
                for (timeout = 0; timeout < ONE_MS_ACTIVE_WAIT; timeout++)
                {
				 	enable = mipi_dsih_dphy_status(instance, clk_lane_mask);
                }
            }
		if (mipi_dsih_dphy_status(instance, clk_lane_mask) != clk_lane_mask)
		{
			return ERR_DSI_TIMEOUT;
        }
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0, 1, 1);
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0, 2, 1);
    }
	return OK;
}
#else
void mipi_dsih_dphy_ulps_clk_lane(dphy_t * instance, int enable)                                                                                                       
{                                                                                                                                                                      
    int timeout;                                                                                                                                                       
    if (enable)                                                                                                                                                        
    {                                                                                                                                                                  
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0, 0, 1);                                                                                          
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 1, 1, 1);                                                                                          
    }                                                                                                                                                                  
    else                                                                                                                                                               
    {                                                                                                                                                                  
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 1, 2, 1);                                                                                          
        for (timeout = 0; timeout < DSIH_PHY_ACTIVE_WAIT; timeout++)                                                                                                   
        {                                                                                                                      
                                                                                                                                               
            if (mipi_dsih_dphy_status(instance, 0x0004) == 0)                                                                                                          
            {                                                                                                                                   
                for (timeout = 0; timeout < ONE_MS_ACTIVE_WAIT; timeout++)                                                                                             
                {                                                                                                                                                      
                    ;                                                                                                                                                  
                }                                                                                                                                                      
                break;                                                                                                                                                 
            }                                                                                                                                                          
        }                                                                                                                                                              
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0, 1, 1);                                                                                          
        mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_IF_CTRL, 0, 2, 1);                                                                                          
    }                                                                                                                                                                  
}
#endif
uint32_t mipi_dsih_dphy_status(dphy_t * instance, uint16_t mask)                                                                                                       
{                                                                                                                                                                      
    return mipi_dsih_dphy_read_word(instance, R_DSI_HOST_PHY_STATUS) & mask;                                                                                           
}                                                                                                                                                                      
void mipi_dsih_dphy_test_clock(dphy_t * instance, int value)                                                                                                           
{                                                                                                                                                                      
    mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_TST_CRTL0, value, 1, 1);                                                                                        
}                                                                                                                                                                      
void mipi_dsih_dphy_test_clear(dphy_t * instance, int value)                                                                                                           
{                                                                                                                                                                      
    mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_TST_CRTL0, value, 0, 1);                                                                                        
}                                                                                                                                                                      
void mipi_dsih_dphy_test_en(dphy_t * instance, uint8_t on_falling_edge)                                                                                                
{                                                                                                                                                                      
    mipi_dsih_dphy_write_part(instance, R_DSI_HOST_PHY_TST_CRTL1, on_falling_edge, 16, 1);                                                                             
}                                                                                                                                                                      
uint8_t mipi_dsih_dphy_test_data_out(dphy_t * instance)                                                                                                                
{                                                                                                                                                                      
    return mipi_dsih_dphy_read_part(instance, R_DSI_HOST_PHY_TST_CRTL1, 8, 8);                                                                                         
}                                                                                                                                                                      
void mipi_dsih_dphy_test_data_in(dphy_t * instance, uint8_t test_data)                                                                                                 
{                                                                                                                                                                      
    mipi_dsih_dphy_write_word(instance, R_DSI_HOST_PHY_TST_CRTL1, test_data);                                                                                          
}                                                                                                                                                                      
/**
 * Write to D-PHY module (encapsulating the digital interface)
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param address offset inside the D-PHY digital interface
 * @param data array of bytes to be written to D-PHY
 * @param data_length of the data array
 */
void mipi_dsih_dphy_write(dphy_t * instance, uint8_t address, uint8_t * data, uint8_t data_length)                                                                     
{                                                                                                                                                                      
    unsigned i = 0;                                                                                                                                                    
    if (data != 0)                                                                                                                                                     
    {                                                                                                                                                                  
#if ((defined DWC_MIPI_DPHY_BIDIR_TSMC40LP) || (defined DPHY2Btql) || (defined GEN_2))
                                                                                      
        mipi_dsih_dphy_test_clock(instance, 1);                                                                                                                        
                                                                                                    
        mipi_dsih_dphy_test_data_in(instance, address);                                                                                                                
                                                                                                                                           
        mipi_dsih_dphy_test_en(instance, 1);                                                                                                                           
                                                                 
        mipi_dsih_dphy_test_clock(instance, 0);                                                                                                                        
                                                                                                 
        mipi_dsih_dphy_test_en(instance, 0);                                                                                                                           
                                                                                                                                          
        for (i = data_length; i > 0; i--)                                                                                                                              
        {                                                                           
            mipi_dsih_dphy_test_data_in(instance, data[i - 1]);                                                                                                        
                                                             
            mipi_dsih_dphy_test_clock(instance, 1);                                                                                                                    
            mipi_dsih_dphy_test_clock(instance, 0);                                                                                                                    
        }                                                                                                                                                              
#endif                                                                                                                                                                 
    }                                                                                                                                                                  
}                                                                                                                                                                      
                                                                                                                                                                       
                                                                                                                                                                       
                                                                                                                                                                       
                                                                                                                                                  
void mipi_dsih_dphy_write_word(dphy_t * instance, uint32_t reg_address, uint32_t data)                                                                                 
{                                                                                                                                                                      
    if (instance->core_write_function != 0)                                                                                                                            
    {                                                                                                                                                                  
        instance->core_write_function(instance->address, reg_address, data);                                                                                           
    }                                                                                                                                                                  
}                                                                                                                                                                      
/**
 * Write bit field to D-PHY module (encapsulating the bus interface)
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param reg_address offset
 * @param data bits to be written to D-PHY
 * @param shift from the right hand side of the register (big endian)
 * @param width of the bit field
 */
void mipi_dsih_dphy_write_part(dphy_t * instance, uint32_t reg_address, uint32_t data, uint8_t shift, uint8_t width)                                                   
{                                                                                                                                                                      
    uint32_t mask = 0;                                                                                                                                                 
    uint32_t temp = 0;                                                                                                                                                 
    if (instance->core_read_function != 0)                                                                                                                             
    {                                                                                                                                                                  
        mask = (1 << width) - 1;                                                                                                                                       
        temp = mipi_dsih_dphy_read_word(instance, reg_address);                                                                                                        
        temp &= ~(mask << shift);                                                                                                                                      
        temp |= (data & mask) << shift;                                                                                                                                
        mipi_dsih_dphy_write_word(instance, reg_address, temp);                                                                                                        
    }                                                                                                                                                                  
}                                                                                                                                                                      
uint32_t mipi_dsih_dphy_read_word(dphy_t * instance, uint32_t reg_address)                                                                                             
{                                                                                                                                                                      
    if (instance->core_read_function == 0)                                                                                                                             
    {                                                                                                                                                                  
        return ERR_DSI_INVALID_IO;                                                                                                                                     
    }                                                                                                                                                                  
    return instance->core_read_function(instance->address, reg_address);                                                                                               
}                                                                                                                                                                      
/**
 * Read bit field from D-PHY module (encapsulating the bus interface)
 * @param instance pointer to structure which holds information about the d-phy
 * module
 * @param reg_address offset
 * @param shift from the right hand side of the register (big endian)
 * @param width of the bit field
 * @return data bits to be written to D-PHY
 */
uint32_t mipi_dsih_dphy_read_part(dphy_t * instance, uint32_t reg_address, uint8_t shift, uint8_t width)                                                               
{                                                                                                                                                                      
    return (mipi_dsih_dphy_read_word(instance, reg_address) >> shift) & ((1 << width) - 1);                                                                            
}                                                                                                                                                                      
                                                                                                                                                                             
