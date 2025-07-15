#ifndef DRONE_H
#define DRONE_H

class Drone {
    public :
    Point position;
    Point last;
    double nabla ; //just the gradient angle, not the slope
    int inside, numCross ;
    bool droneIn;
    double angleTurned;
    double distTraversed;
    int currentGaussian;
    Point motion; // FIX ME ; can delete
    vector<Point> polytope;
    vector<double> lastContourGradient;
    vector<double> currentContourGradient;
    LineSegment currPath;
    vector<double> lastTangent;
    vector<double> currentTangent;
    int side;
    
    Drone() 
    : position(Point(0, 0)), 
      last(Point(0, 0)), 
      inside(0), 
      nabla(0.0), 
      droneIn(false), 
      angleTurned(0.0), 
      distTraversed(0.0), 
      numCross(0), 
      currentGaussian(0), 
      lastContourGradient(), 
      currentContourGradient(), 
      motion(Point(0,0)),
      currPath(Point(0,0), Point(0,0)),
      side(0) {}

    Drone(Point P1, Point P2, int side, double nab, bool flag) 
        : position(P1), 
        last(P2), 
        inside(0), 
        nabla(nab), 
        droneIn(flag), 
        angleTurned(0.0), 
        distTraversed(0.0), 
        numCross(0), 
        currentGaussian(0), 
        lastContourGradient(), 
        currentContourGradient(), 
        motion(Point(0,0)),
        currPath(Point(0,0), Point(0,0)),
        side(side) {}
    
    //this appears to be a broken implementation of this function but we need it to compile
    // MoveDrone 0
    bool MoveDrone(Param* p, double alpha, double dist, PLUME &plume, int callSource)
    {
        Point nextPosition;
        vector<Point> points;
        points.push_back(position);
        
        motion = PointUtil::vector(nabla + p->alpha, dist);
        nextPosition = position + motion;
        
        fprintf(p->out,"MoveDrone0 %x\n%f %f\n",this,nextPosition.x,nextPosition.y);
        points.push_back(nextPosition);
        LineSegment dronemotion = LineSegment(position, nextPosition);
        
        distTraversed += dist;
        
        if (abs(nextPosition.x) > 3 || abs(nextPosition.y) > 3)
            cout << "position exception! "<<endl;
        
       // bool cross = plume.crossesEdge (dronemotion);
        CrossData cd = plume.getCross(p, dronemotion, nabla, p->alpha, dist, inside);
      //  CrossData cd = plume.getCross(p, dronemotion);
        Point crossingPoint = cd.first;
        
        points.push_back(crossingPoint);
        
        swap(points[1], points[2]);
        int cross = cd.second; //changed to boolean if 1 then crossed, 0 did not cross.
     //   if (abs(crossingPoint.x) > 1e-9 || abs (crossingPoint.y) > 1e-9)
       //     cross = 1;
       // if (cross)
        //    cout<<cross<<  " cross "<<crossingPoint.x << " " <<crossingPoint.y<<endl;
        if (cross)
        {
            ++numCross;
            inside = inside ^ 1;
           // if (abs(p->alpha) > p->epsilon)
             //   cout<<"Testing p->alpha " << inside<<endl;
            vector<double> gradient_vector;
            
          //  if (cd.second != currentGaussian1)
            //    gradient_vector = get_Gaussian_vector( points, cd.second);
           // else
                gradient_vector = gradient_LSQ(p, points);
            
       //     last_gradient = gradient_vector;
       //     Point gradient_shift = PointUtil::vector(1.0/100,1);
       //     last_gradient[0] += gradient_shift.x;
       //     last_gradient[1] += gradient_shift.y;
            double angle = getAngle(gradient_vector);

            
           // if (!callSource)
             //   cout<<"Called from CrossPlume "<<endl;
            
            angle = gradient_modulo(angle);
            if (droneIn)
            {
          //      cout<<gradient_vector[0]<<" "<<gradient_vector[1] << " gradient vector,Points "<<crossingPoint.x<<" "<<crossingPoint.y<<endl;
             //   cout<<position.x<<" "<<position.y<<" "<<nextPosition.x<<" "<<nextPosition.y<<endl;
            }
            double gradient = angle + PI/2 ;
       //     cout<<angle<<" angle "<<points.size()<<" "<<gradient<< endl;
            gradient = gradient_modulo(gradient);
            
            Point checkPoint = PointUtil::vector(gradient, dist);
           
      //      cout<<"Checkpoint "<<checkPoint.x<<" "<<checkPoint.y<<endl;

            checkPoint = crossingPoint + checkPoint;
           
            
        //    cout<<"Checkpoint "<<checkPoint.x<<" "<<checkPoint.y<<endl;
            
            int orient ;
            if (inside)
                orient = PointUtil::CLOCKWISE;
            else
                orient = PointUtil::COUNTERCLOCKWISE;
           // else
             //   orient = (inside) ? (PointUtil::COUNTERCLOCKWISE) : (PointUtil::CLOCKWISE);
            
          //  if (callSource == 0)
        //        orient = orient ^ 1;
            
            
            if (PointUtil::orientation (position, crossingPoint, checkPoint) != orient)
                reverse (gradient);

            angleTurned += changeGradient(nabla + p->alpha, gradient);
            nabla = gradient;
            
        //    if (droneIn && !inside)
          //          cout << crossingPoint.x <<  " crossed here "<< crossingPoint.y <<  " " <<droneIn<<" inside: "<<inside<<" "<<" "<< gradient<< endl;
         }
        last = position;
        position = nextPosition;
        if (numCross)
            polytope.push_back (position);
      //  if (numCross == 1 && cross)
        //    polytope.push_back (cd.first);

    
        if (polytope.size() > 1)
        {
            int N = polytope.size();
       //     if (droneIn)
         //       fprintf (out, "Line (%lf,%lf) (%lf,%lf)\n", polytope[N-2].x, polytope[N-2].y, polytope[N-1].x, polytope[N-1].y);
            Point currtoinit = polytope.back() - polytope[0];
            
            
            return (currtoinit.length() < p->INF) && (numCross > p->CROSSBOUND);
        }
        else
            return false ;
    }

    // Fixed implementation
    // MoveDrone 1
    void MoveDrone(Param* p, double alpha, double dist, 
                                    double diffepsilon, int callSource)
    {
        Point nextPosition;
    
        // Calculate motion vector and next position
        motion = PointUtil::vector(nabla + p->alpha, dist);
        nextPosition = position + motion;
        
        fprintf(p->out,"MoveDrone1 %x\n%f %f\n",this,nextPosition.x,nextPosition.y);
        // points.push_back (nextPosition);
        currPath = LineSegment(position, nextPosition);
        
        // Update distance traversed
        distTraversed += dist;
        
        // Check for position exception
        if (abs(nextPosition.x) > 5 || abs(nextPosition.y) > 5){
            //cout << "position exception! "<<endl;
        }
        last = position;
        position = nextPosition;
        polytope.push_back (position);

        //cout << "moved one drone now getting contour grad" << endl;
        // Calculate gradient using criticalPath
        vector<Point> surroundingPoints;
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                if (i == 0 && j == 0) continue; 
                if (i != 0 && j != 0) { 
                    // FIX ME: do we use diff epsilon here?
                    Point offset = Point(i * diffepsilon, j * diffepsilon);
                    surroundingPoints.push_back(position + offset);
                }
            }
        }

        vector<double> gradient = concentration_gradient_LSQ(p, surroundingPoints, position);
        // cout << "gradient size : " << gradient.size() << endl;
        // for(int i = 0; i < gradient.size(); i++){
        //     cout << "gradient: [" << i << "] " << gradient[i] << endl;
        // }
        if(lastContourGradient.size() > 0){
            lastContourGradient = currentContourGradient;
        }else{
            vector<Point> surroundingPointsLast;
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (i == 0 && j == 0) continue; 
                    if (i != 0 && j != 0) { 
                        // FIX ME: do we use diff epsilon here?
                        Point offset = Point(i * diffepsilon, j * diffepsilon);
                        surroundingPointsLast.push_back(last + offset);
                    }
                }
            }

            lastContourGradient = concentration_gradient_LSQ(p, surroundingPointsLast, last); 
        }
        currentContourGradient = gradient;

        //cout << "current point: " << position.x << " " << position.y << endl;
    }

#ifndef LEGACY
    
    void LearnGradient(Param* p, double alpha, double dist, Point crossingPoint, Drone &otherDrone, vector<double> gradient_vector) {
        //cout << "Learning gradient for drone pair" << endl;
        
        //cout << "gradient vector: " << gradient_vector[0] << " " << gradient_vector[1] << endl;
        double angle = 0;
        if( ( gradient_vector[0] != 0 ) || ( gradient_vector[1] != 0 ) ){
          angle = getAngle(gradient_vector);
        }

        assert(abs(angle) < (2*PI) );

        //out << "angle: " << angle << endl;

        //double gradient = angle + PI/2 ;
        double gradient = angle;
        gradient = gradient_modulo(gradient);
        Point checkPoint = PointUtil::vector(gradient, dist);
        checkPoint = crossingPoint + checkPoint;
        /* 
        // FIX ME: double check this
        int orient;
        if (side == 2) // 2 is RIGHT
            orient = PointUtil::CLOCKWISE;
        else
            orient = PointUtil::COUNTERCLOCKWISE;
        
        Point curr =  position;
        if (PointUtil::orientation (curr, crossingPoint, checkPoint) != orient)
            reverse (gradient);

        */

        angleTurned += changeGradient(nabla + p->alpha, gradient);
        nabla = gradient;
            
    }

};
#endif //LEGACY

#ifdef LEGACY
    // This function is called when the drones cross the critical path
    // and need to learn the gradient (update nabla)
    void LearnGradient(Param* p, double alpha, double dist, Point crossingPoint, Drone &otherDrone, vector<double> gradient_vector) {
        cout << "Learning gradient for drone pair" << endl;
        
        cout << "gradient vector: " << gradient_vector[0] << " " << gradient_vector[1] << endl;

        double angle = getAngle(gradient_vector);


        angle = gradient_modulo(angle);
        cout << "angle: " << angle << endl;

        double gradient = angle + PI/2 ;
        gradient = gradient_modulo(gradient);
        Point checkPoint = PointUtil::vector(gradient, dist);
        checkPoint = crossingPoint + checkPoint;
                
        // FIX ME: double check this
        int orient;
        if (side == 2) // 2 is RIGHT
            orient = PointUtil::CLOCKWISE;
        else
            orient = PointUtil::COUNTERCLOCKWISE;
        
        Point curr =  position;
        if (PointUtil::orientation(curr, crossingPoint, checkPoint) != orient)
            reverse (gradient);

        angleTurned += changeGradient(nabla + p->alpha, gradient);
        nabla = gradient;
            
    }

};
#endif //LEGACY

#endif //DRONE_H
