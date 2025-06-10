
void Sync (Drone &A, Drone &B, double alpha, double dist, PLUME &plume)
{
    Point v;
    
    if (alpha > 0)
        v = PointUtil::vector (A.nabla + alpha - PI/2, DIST * epsilon);
    else
        v = PointUtil::vector (A.nabla + alpha + PI/2, DIST * epsilon);
    
    Point nextPosition = A.position + v;
    B.last = nextPosition;
    B.position = nextPosition;
    LineSegment dronemotion = LineSegment (B.last, B.position);
 //   if (abs(B.nabla - A.nabla) < 1e-9 &&  plume.getCross(dronemotion, B.nabla, alpha, dist, B.inside).second == 1)
   //     B.inside = B.inside ^ 1;
    B.nabla = A.nabla;
   // B.inside = A.inside;
    B.polytope.push_back (nextPosition);
    //ignoring angle turned during sync.
    
    return ;
}

// FIX ME: Verify this works for the critical path tracing
void Sync (Drone &A, Drone &B, double alpha, double dist, criticalPath &cp)
{
    Point v;
    
    if (alpha > 0)
        v = PointUtil::vector (A.nabla + alpha - PI/2, DIST * epsilon);
    else
        v = PointUtil::vector (A.nabla + alpha + PI/2, DIST * epsilon);

    
    Point nextPosition = A.position + v;
    B.last = nextPosition;
    B.position = nextPosition;


    Point BLast = B.last;
    Point BPosition = B.position;

    LineSegment dronemotion = LineSegment(BLast, BPosition);   

    B.nabla = A.nabla;
    B.polytope.push_back (nextPosition);

    //ignoring angle turned during sync.
    cout << "exiting sync" << endl;
    return ;
}

/***********************************************************/

void print_data(Drone A, Drone B)
{
    cout << "printing data..." << endl;
    int numPoints = A.polytope.size();
    
    cout << "printing data 1..." << endl;
    fprintf (out, "Pen b\n");
    
    for (int i = 0;i < numPoints; i ++)
        fprintf (out, "Line (%lf,%lf) (%lf,%lf)\n", A.polytope[i].x, A.polytope[i].y, A.polytope[(i+1)%numPoints].x, A.polytope[(i+1)%numPoints].y);
  
    numPoints = B.polytope.size();
  
    cout <<numPoints << endl;

//    fprintf (out, "Ellipse (%lf,%lf) %lf %lf \n", plume.ovals[0].center.x, plume.ovals[0].center.y, R, R/2);
 //   fprintf (out, "Ellipse (%lf,%lf) %lf %lf \n", plume.ovals[1].center.x, plume.ovals[1].center.y, R, R/2);

    cout << "printing data 2..." << endl;
    fprintf (out, "Pen r\n");
    
    for (int i = 0;i < numPoints; i ++)
        fprintf (out, "Line (%lf,%lf) (%lf,%lf)\n", B.polytope[i].x, B.polytope[i].y, B.polytope[(i+1)%numPoints].x, B.polytope[(i+1)%numPoints].y);
 
    cout << "done printing data..." << endl;
    return ;
}


bool CrossPlume (Drone &A, Drone &B, double alpha, PLUME &plume)
{
    Point start_pos = A.last;
   // if (abs(start_pos.x) < 2 && abs(start_pos.y) < 2)
     //   cout <<"start pos ... "<< start_pos.x << " " << start_pos.y <<" "<<check(A.position)<< endl;

    int crossing = A.inside;
    bool orient = true ;
    double alphainitial = alpha;
    bool endHere = false;
    
    int iterate = 1;
    
    do{
        endHere = endHere || A.MoveDrone (alpha, epsilon * epsilon, plume, 0);
     //   B.MoveDrone (alpha, epsilon * epsilon, plume, 0);

#ifndef LEGACY
    fprintf(out,"distance from droneA to source %f\n",get_dist(gaussianCenter[0],A.position));//do we converge?
#endif
        
        if (PointUtil::orientation(A.last, A.position, start_pos) == PointUtil::CLOCKWISE && alpha > 0)
            orient = false ;
        if (PointUtil::orientation(A.last, A.position, start_pos) == PointUtil::COUNTERCLOCKWISE && alpha < 0)
            orient = false ;
      //  if (crossing == A.inside)
        //    Sync (A,B,alpha, epsilon * epsilon, plume);
      //  else
        //    Sync (A,B,alphainitial, epsilon * epsilon, plume);
        
        if (alpha > 0)
            alpha += epsilon;
        else
            alpha -= epsilon;
        A.angleTurned += abs (alphainitial);
     //   cout << "testing cross plume ... "<< A.position.x << " " << A.position.y << endl;
     //   cout << "testing cross plume ... "<< B.position.x << " " << B.position.y << endl;
        iterate--;
        if (iterate < 0){
            print_data (A,B);
            exit (0);
        }
       // cout<<crossing<<" "<<A.inside<<" "<<A.droneIn<<endl;
    }while ( (crossing == A.inside) && orient && (!endHere) );
        
    if ( (crossing == A.inside) && (!endHere) ){
        A.polytope.pop_back ();
        A.position = A.last;
        A.angleTurned -= abs (alphainitial);
        double dx = start_pos.getX () - A.position.getX();
        double dy = start_pos.getY () - A.position.getY();
        double gradient = atan2 (dy, dx);
       
    //    A.position = A.last;
     //   B.position = B.last;
        
        Point d1 = start_pos - A.position;
        Point motion;
        if (d1.length() > (epsilon*epsilon) )
            motion = PointUtil::vector (gradient, epsilon * epsilon);
        else
            motion = PointUtil::vector (gradient, d1.length());
        motion = A.position + motion;
        Point d2 = start_pos - motion;
     //   cout<<A.position.x<<" " <<A.position.y<<" "<<d1.length()<< " distances " << d2.length()<<endl;

        if (d1.length() < d2.length()){
            reverse (gradient);
        }
        A.angleTurned += changeGradient (A.nabla + alpha, gradient);
        A.nabla = gradient;
     //   B.nabla = gradient;
     //   cout << start_pos.x << " starting here " << start_pos.y << " "<<gradient<<" "<<check (start_pos) <<" "<<check(A.position)<<" "<<A.inside<< endl;
     //   cout << "testing cross plume2 ... "<< A.position.x << " " << A.position.y << endl;
     //   cout << "testing cross plume2 ... "<< A.last.x << " " << A.last.y << endl;
        int iter = 0;
        while ( (crossing == A.inside) && (!endHere) ){
            d1 = start_pos - A.position;
            if (d1.length() > (epsilon * epsilon) )
            {
                endHere = endHere || A.MoveDrone (0, epsilon*epsilon, plume, 0);
           //     Sync (A,B,alphainitial, epsilon * epsilon, plume);

            //    B.MoveDrone (0, epsilon * epsilon, plume, 0);
             //   B.MoveDrone (0, epsilon*epsilon, plume);
            }
            else{
                endHere = endHere || A.MoveDrone (0, d1.length(), plume, 0);
            //    Sync (A,B,alphainitial, d1.length(), plume);

             //   B.MoveDrone (0, d1.length(), plume, 0);

            //    B.MoveDrone (0, d1.length()/2, plume);
            }
            
         //   if (abs(A.position.x) > 2 || abs(A.position.y) > 2)
           //     break ;
            ++iter;
            if (iter > 1)
            {
                cout<<"Iterations exceeding ..."<<endl;
                print_data(A,B);
                exit (0);
            }
        //    Sync2 (A,B, alpha);
            //cout << "testing cross plume2 ... "<< A.position.x << " " << A.position.y << endl;
      //      cout << "testing cross plume2 ... "<< B.position.x << " " << B.position.y << endl;
        }
      //  B.nabla = A.nabla;
     //   Sync (A,B,alphainitial, epsilon * epsilon, plume);
    }
    
    return endHere;
}


// FIX ME: Make this work for the critical path tracing
bool CrossCriticalPath(Drone &A, Drone &B, double alpha, criticalPath &cp)
{
    Point start_pos = A.last; 

    int crossing = A.side; 
    bool orient = true ;
    double alphainitial = alpha;
    bool endHere = false;
    
    int iterate = 1;
    

    do{
        A.MoveDrone (alpha, epsilon, cp.diffepsilon, 0);

#ifndef LEGACY
    fprintf(out,"distance from droneA to source %f\n",get_dist(gaussianCenter[0],A.position));//do we converge?
#endif

        CrossData crossData = cp.getCross(A, B, alpha, epsilon);
        if(crossData.second){
            vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
            A.LearnGradient(alpha, epsilon, crossData.first, B, gradient_vec);
            B.LearnGradient(alpha, epsilon, crossData.first, A, gradient_vec);
        }
        cout << "drone side? " << A.side << endl;
        endHere = endHere || cp.foundSource(A, B);
        
        // FIX ME: check if this is determining the correct orientation?
        Point last = A.last;
        Point position = A.position;
        if ( ( PointUtil::orientation(last,position, start_pos) == PointUtil::CLOCKWISE ) && (alpha > 0) )
            orient = false ;
        if ( (PointUtil::orientation(last, position, start_pos) == PointUtil::COUNTERCLOCKWISE) && (alpha < 0) )
            orient = false ;

        if (alpha > 0)
            alpha += epsilon;
        else
            alpha -= epsilon;

        A.angleTurned += abs (alphainitial);

        iterate--;
        if (iterate < 0){
            print_data (A,B);
            exit (0);
        }
    }while ( (crossing == A.side) && orient && (!endHere) );
        
    if ( (crossing == A.side) && (!endHere) ){
        A.polytope.pop_back (); 
        A.polytope.pop_back();
        A.position = A.last;
        A.angleTurned -= abs (alphainitial);
        double dx = start_pos.getX () - A.position.getX();
        double dy = start_pos.getY () - A.position.getY();
        double gradient = atan2 (dy, dx);
        
        Point pos = A.position;
        Point d1 = start_pos - pos;
        Point motion;
        if (d1.length() > epsilon*epsilon)
            motion = PointUtil::vector (gradient, epsilon * epsilon);
        else
            motion = PointUtil::vector (gradient, d1.length());
        motion = pos + motion;
        Point d2 = start_pos - motion;

        if (d1.length() < d2.length()){
            reverse (gradient);
        }

        A.angleTurned += changeGradient (A.nabla + alpha, gradient);
        A.nabla = gradient;

        int iter = 0;
        while ( (crossing == A.side) && (!endHere) ){
            Point new_pos = A.position; 
            d1 = start_pos - new_pos; 
            if (d1.length() > (epsilon * epsilon) )
            {
                A.MoveDrone(0, epsilon*epsilon, cp.diffepsilon, 0);
#ifndef LEGACY
    fprintf(out,"distance from droneA to source %f\n",get_dist(gaussianCenter[0],A.position));//do we converge?
#endif
                CrossData crossData = cp.getCross(A, B, alpha, epsilon);
                if(crossData.second){
                    vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
                    A.LearnGradient(alpha, epsilon, crossData.first, B, gradient_vec);
                    B.LearnGradient(alpha, epsilon, crossData.first, A, gradient_vec);
                }
                cout << "Drone side now " << A.side << endl;
                endHere = endHere || cp.foundSource(A, B); 
            }
            else{
                A.MoveDrone(0, d1.length(), cp.diffepsilon, 0);
#ifndef LEGACY
    fprintf(out,"distance from droneA to source %f\n",get_dist(gaussianCenter[0],A.position));//do we converge?
#endif
                CrossData crossData = cp.getCross(A, B, alpha, epsilon);
                if(crossData.second){
                    vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
                    A.LearnGradient(alpha, epsilon, crossData.first, B, gradient_vec);
                    B.LearnGradient(alpha, epsilon, crossData.first, A, gradient_vec);
                }
                cout << "Drone side now " << A.side << endl;
                endHere = endHere || cp.foundSource(A, B);
            }
            
            ++iter;
            if (iter > 1)
            {
                cout<<"Iterations exceeding ..."<<endl;
                print_data(A,B);
                exit (0);
            }
        }
    }
    
    return endHere;
}

/***********************************************************************/
void sketch_algorithm ()
{
    
    Drone A (drone_start_A, drone_start_A, 1, 0, true);
    Drone B (drone_start_B, drone_start_B, 2, 0, false);
    print_test_infrastructure_info(); //print initialization stats
    
    bool loopEnd = false ;

    Point startPoint = {(drone_start_A.x + drone_start_B.x) / 2, (drone_start_A.y + drone_start_B.y) / 2};
    double res = epsilon*epsilon; // how much resolution should we calculate contourlines at?
    criticalPath cp(startPoint, epsilon, res);
        int iter = 0; // delete before push
    
    //lvl 0 
    do{

        /******************************/
        /* try to catch signals
         * currently fails because we are stuck inside one of the functions called later in this loop
         * use ctrl+z to pause and kill to kill the process if other signals fail
         */
        /*if(PyErr_CheckSignals()){
            throw py::error_already_set();
        }*/
        /******************************/

        int iter = 0; //uncomment before push

        //lvl 1
        while (( (A.numCross + B.numCross == 0) || (3 == A.side + B.side) ) && (!loopEnd) )
        {
            cout << "inside loop " << iter << endl;
            ++iter;
            if (iter > 100)
            {
                cout<<"Iterations exceeding ..."<<endl;
                print_data(A,B); 
                exit (0);
            }
            
            A.MoveDrone(alpha, epsilon, cp.diffepsilon, 1);
            B.MoveDrone(alpha, epsilon, cp.diffepsilon, 1);

#ifndef LEGACY
    fprintf(out,"distance from droneA to source %f\n",get_dist(gaussianCenter[0],A.position));//do we converge?
#endif

            loopEnd = loopEnd || cp.foundSource(A, B);
            
            CrossData crossData = cp.getCross(A, B, alpha, epsilon);
            if(crossData.second){
                // cout << "numcross A" << A.numCross << endl;
                // cout << "numcross B" << B.numCross << endl;
                // cout << "A side" << A.side << endl;
                // cout << "B side" << B.side << endl;

                // cout << "crossed; learning gradient..." << endl;
                vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
                A.LearnGradient(alpha, epsilon, crossData.first, B, gradient_vec);
#ifndef LEGACY
                /*****************************************
                 * this is a simple test that is accurate when there is only one gaussian
                 * we compute the shortest path between the drone and the center of the gaussian
                 * and take that to be the real gradient.
                 * Then we can try to compute the error.
                 * I'm concerned about the use of polar coordinates because it introduces additional
                 * complexity compared to the alternatives that we could use in cartesian coordinates
                 * such as: 
                 *   store the normalized grad vector, or normalize to either the x or y component
                 * 
                 * nabla is supposed to store the angle of the gradient(scalar)
                 * */
                fprintf(out,"distance from droneA to source %f\n",get_dist(gaussianCenter[0],A.position));//printf drone tracking
                Point realgrad = Point(gaussianCenter[0].x-A.position.x,
                gaussianCenter[0].y-A.position.y);
                Point graderr = Point(realgrad.x - gradient_vec[0], realgrad.y - gradient_vec[1]);
                fprintf(out,"real grad: %f, %f\n grad_approx: %f, %f\n grad_error: %f,%f\n",
                    realgrad.x,realgrad.y,
                    gradient_vec[0], gradient_vec[1],
                    graderr.x, graderr.y);

                vector<double> rgrad;
                rgrad.push_back(realgrad.x);
                rgrad.push_back(realgrad.y);
                double realgradangle = getAngle(rgrad);
                fprintf(out,"nabla %f realgrad angle %f\n",
                    A.nabla,
                    realgradangle);
                      
                fflush(out);
                /******************************************/
#endif //LEGACY
                B.LearnGradient(alpha, epsilon, crossData.first, A, gradient_vec);

            }

            
        }//lvl1


        if ( (A.side + B.side) != 3)
        {

            // If A crosses
            if ( (A.side == 2) && (B.side == 2) )
            {
                alpha = -epsilon;
            //    B.nabla = A.nabla;

                loopEnd = loopEnd || CrossCriticalPath (B,A, alpha, cp);
                cout << "about to sync" << endl;
                Sync (B,A,alpha, epsilon, cp); // FIX ME
                A.nabla = B.nabla;
              
 //               cout << "testing Sync A... "<< A.position.x << " "<<A.position.y <<" "<<alpha<<" "<<A.nabla<< endl;
 //               cout << "testing Sync B... "<< B.position.x << " "<<B.position.y <<" "<<alpha<<" "<<B.nabla<< endl;
            }
            else // B crosses
            {
              alpha = epsilon;
              loopEnd = loopEnd || CrossCriticalPath (A,B, alpha, cp);
              cout << "about to sync" << endl;
              Sync (A,B,alpha, epsilon, cp);
              B.nabla = A.nabla;
                
   //             cout << "testing Sync B... "<< B.position.x << " "<<B.position.y <<" "<<alpha<<" "<<B.nabla<< endl;
   //             cout << "testing Sync A... "<< A.position.x << " "<<A.position.y <<" "<<alpha<<" "<<A.nabla<< endl;
            
            }
        }
    }while (!loopEnd);//lvl 0
    

    if (!A.polytope.empty()) {
        cout <<"Initial crossing with A is " << A.polytope[0].getX() << " " << A.polytope[0].getY() << endl;
    } else {
        cout << "Initial crossing with A is not available as polytope is empty." << endl;
    }
    cout << "angle turned by A is " << " " << A.angleTurned << endl;
    cout << "distance traversed by A is "<< " " << A.distTraversed << endl;
    if (!A.polytope.empty()) {
        cout << "area estimated by A is " << " " << estimateArea (A.polytope) << endl;
        areas.push_back (estimateArea (A.polytope));
    } else {
        cout << "area estimated by A is not available as polytope is empty." << endl;

    lengths.push_back (A.distTraversed);
    angles.push_back (A.angleTurned);
    cout << "actual area is  " << PI * majorAxis * minorAxis << endl;
    }

    print_data (A,B);
    
    return ;
}   


void test_infrastructure()
{
    gaussianCenter.clear();
    gaussianVar.clear();
    //initial direction
    alpha = 1.47; // slightly off pi/2 to try to cause early crossing
    //velocity
    epsilon = 0.1;

    CROSSBOUND = 100;
    majorAxis = 0.25;
    minorAxis = 0.25;
    
    DIST = sqrt (49);
    
    THRESHOLD = exp (-majorAxis*majorAxis);
   
    drone_start_B = Point (1 + DIST*epsilon*0.5,-2.38);
    drone_start_A = Point (1 - DIST*epsilon*0.5,-2.38);

    drone_start_BB = Point (1 + DIST*epsilon*0.5,-2.38);
    drone_start_AB = Point (1 - DIST*epsilon*0.5,-2.38);
   
#ifndef LEGACY
    num = 1;
    gaussianCenter.push_back(Point(1,1));
    gaussianVar.push_back(Point(1,1));
#endif //LEGACY
#ifdef LEGACY
    // data/gaussian_contours_data_7.txt  
    num = 4;
    gaussianCenter.push_back(Point(1,1));
    gaussianCenter.push_back(Point(-2,0));
    gaussianCenter.push_back(Point(1,0));
    gaussianCenter.push_back(Point(4,0));
    
    gaussianVar.push_back(Point(0.6,0.6));
    gaussianVar.push_back(Point(0.8,1.0));
    gaussianVar.push_back(Point(0.9,1.8));
    gaussianVar.push_back(Point(0.5,0.5));
#endif //LEGACY
    print_test_infrastructure_info(); //print initialization stats
        
}

