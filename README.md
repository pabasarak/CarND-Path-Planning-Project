# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
## The main purpose of the code is to 
1)	Drive 50MPS when ever its possible
2)	If there is a slow moving vehicle ahead, shift lane without colliding to other vehicles
3)	If lane shift is not possible, slow down and avoid collusion.

## 1)	Driving at 50MPS
Way points are given to the vehicle such that 50MPS is achieved with a acceleration of 1ms^2. 


          if(too_close)
          	{
          	  ref_vel -= 0.224*30/front_clearance;
          	  
          	}
          	else{
          	
          	  if(ref_vel < 49)
          	  {
          	    ref_vel += 0.224;
          	  }
          	
          	}

## 2)	Sensor Fusion and tracking surrounding vehicles
With sensor fusion data, go through all vehicles and find out vehicles moving along 3 lanes which are between 20m behind and 30m ahead of the ego car. 
a.	If a vehicle in the same lane, we soon we become close to that vehicle and too close flag is set.
b.	If a vehicle in left or right of us, flags are set indicating there is a vehicle to our left or right.
c.	If we are too close, and if there are is a vacant lane in left or right, shift to that lane
d.	If we are too close and there are no vacant lanes, slow down

            if( (check_car_s - car_s)< 30)
          	    {
          	      //ref_vel = 29.0;
          	      
          	      if(check_car_s > (car_s) && d <(2+4*lane + 2 )&& d > (2+4*lane-2))      // lane 1 -> 4< d <8
          	  	{
          	      	 too_close = true;
				front_clearance = check_car_s - car_s;

          	      	}
          	      	//check side lane clearance
          	      	if(check_car_s > (car_s-20) &&(lane > 0) && (d <(2+4*lane - 2 ))&& (d > (2+4*lane-6))) //lane 1 -> 0<d<4
          	  	{
          	      	 car_in_left = true;
          	      	}
          	      	
          	      	
          	      	if(check_car_s > (car_s-20) &&(lane < 2) && (d <(2+4*lane + 6 ))&& (d > (2+4*lane+2))) // lane 1 -> 8<d<12
          	  	{
          	      	 car_in_right = true;
          	      	}
          	      	
          	      
          	      
          	    
          	    }
                
                
               if(too_close)
               {
                  if(car_in_right == false && lane < 2)
                  {
                    lane++;
                    too_close= false;
                  }
                  else
                  {
                     if(car_in_left == false && lane >0)
                      {
                        lane--;
                        too_close= false;
                      }

                  }
               }

## 3)	Trajectory generation
a.	Take two points from the previous path

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
      
b.	Add 30,60 and 90m ahead points to that path
      
      vector<double> next_wp0= getXY(car_s +30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		vector<double> next_wp1= getXY(car_s +60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		vector<double> next_wp2= getXY(car_s +90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
      
c.	Shift the car reference angle to 0 degrees

      for(int i =0; i<ptsx.size(); i++)
		{
			// shift the car reference angle to 0 degrees
			
			double shift_x = ptsx[i] - ref_x;
			double shift_y = ptsy[i] - ref_y;
			
			ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
			ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
		
		}
		
d.	Fit spline curve to new points and transform back to original reference angle

      double x_point = x_add_on+(target_x)/N;
		double y_point = s(x_point);
      
      
      x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
		y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
       
       
e.	Add new points to the previous unused path points and generate a new path.

      for(int i = 1; i<=50-previous_path_x.size(); i++)
		{
		
		 double N = (target_dist/(0.02*ref_vel/2.24));
		 double x_point = x_add_on+(target_x)/N;
		 double y_point = s(x_point);
		 
		 x_add_on = x_point;
		 
		 double x_ref = x_point;
		 double y_ref = y_point;
		 
		 x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
		 y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
		 
		 x_point += ref_x;
		 y_point += ref_y;
		
		 next_x_vals.push_back(x_point);
		 next_y_vals.push_back(y_point);
		 
		 
		 
		}
      

      






