#include "logging.h"
#include "stats.h"
#include "utils.h"

#ifndef LEGACY
/**
 * @brief Synchronizes Drone B's position and gradient to follow Drone A.
 *
 * This function computes a new position for Drone B based on Drone A's orientation
 * and the angular offset `alpha`. The new position is set at a distance of
 * `DIST * epsilon` from A in a direction perpendicular to A's current gradient.
 *
 * The direction of offset depends on the sign of alpha:
 *  - If alpha > 0, offset is rotated left (−π/2)
 *  - If alpha < 0, offset is rotated right (+π/2)
 *
 * This function also updates B's gradient to match A's and appends the new position
 * to B's trajectory (`polytope`).
 *
 * @param p        Global parameters
 * @param A        Lead drone (whose path we're syncing to)
 * @param B        Follower drone (being updated)
 * @param alpha    Angular offset for side-stepping
 * @param dist     Step size (usually epsilon or epsilon²)
 * @param env      Environment (either PLUME or criticalPath, currently unused)
 * @param verbose  Optional debug flag to print actions to stdout
 */

template <typename Env>
void Sync(Param* p, Drone& A, Drone& B, double alpha, double dist, Env& env, bool verbose = true)
{
    Point v;
    if (p->alpha > 0) {
        v = PointUtil::vector(A.nabla + p->alpha - PI/2, p->DIST * p->epsilon);
    } else {
        v = PointUtil::vector(A.nabla + p->alpha + PI/2, p->DIST * p->epsilon);
    }

    Point nextPosition = A.position + v;
    B.last = B.position;        // Save previous position if needed
    B.position = nextPosition;
    B.nabla = A.nabla;
    B.polytope.push_back(nextPosition);

    if (verbose) {
        std::cout << "[SYNC] Drone " << B.id << " moved to (" << B.position.x << ", " << B.position.y << ")\n";
        std::cout << "        Based on Drone " << A.id << " with angle " << A.nabla << "\n";
    }
}

#else
void Sync(Param* p, Drone &A, Drone &B, double alpha, double dist, PLUME &plume)
{
    Point v;
    
    if (p->alpha > 0)
        v = PointUtil::vector(A.nabla + p->alpha - PI/2, p->DIST * p->epsilon);
    else
        v = PointUtil::vector(A.nabla + p->alpha + PI/2, p->DIST * p->epsilon);
    
    Point nextPosition = A.position + v;
    B.last = nextPosition;
    B.position = nextPosition;
    LineSegment dronemotion = LineSegment(B.last, B.position);
 //   if (abs(B.nabla - A.nabla) < 1e-9 &&  plume.getCross(dronemotion, B.nabla, p->alpha, dist, B.inside).second == 1)
   //     B.inside = B.inside ^ 1;
    B.nabla = A.nabla;
   // B.inside = A.inside;
    B.polytope.push_back(nextPosition);
    //ignoring angle turned during sync.
    
    return ;
}

// FIX ME: Verify this works for the critical path tracing
void Sync(Param* p, Drone &A, Drone &B, double alpha, double dist, criticalPath &cp)
{
    Point v;
    
    if (p->alpha > 0)
        v = PointUtil::vector(A.nabla + p->alpha - PI/2, p->DIST * p->epsilon);
    else
        v = PointUtil::vector(A.nabla + p->alpha + PI/2, p->DIST * p->epsilon);

    
    Point nextPosition = A.position + v;
    B.last = nextPosition;
    B.position = nextPosition;


    Point BLast = B.last;
    Point BPosition = B.position;

    LineSegment dronemotion = LineSegment(BLast, BPosition);   

    B.nabla = A.nabla;
    B.polytope.push_back(nextPosition);

    //ignoring angle turned during sync.
    cout << "exiting sync" << endl;
    return ;
}
#endif

#ifndef LEGACY
/**
 * @brief Prints the polytope paths of two drones to the output stream.
 *
 * This function writes line segments connecting the points in each drone's 
 * trajectory (stored in their `polytope` vectors) to the output file stream
 * specified in the `Param` struct. Each drone's path is drawn in a different 
 * color using "Pen b" (blue) for Drone A and "Pen r" (red) for Drone B.
 *
 * The output is in a textual line-drawing format, suitable for visualization 
 * tools that parse "Line (x1,y1) (x2,y2)" syntax.
 *
 * @param p Pointer to global parameters, including the output file stream
 * @param A The first drone (blue path)
 * @param B The second drone (red path)
 */
void print_data(Param* p, const Drone& A, const Drone& B)
{
    auto print_polytope = [&](const vector<Point>& poly, const char* pen) {
        fprintf(p->out, "Pen %s\n", pen);
        int n = poly.size();
        for (int i = 0; i < n; ++i) {
            const Point& curr = poly[i];
            const Point& next = poly[(i + 1) % n];
            fprintf(p->out, "Line (%lf,%lf) (%lf,%lf)\n", curr.x, curr.y, next.x, next.y);
        }
    };

    cout << "Printing data..." << endl;
    print_polytope(A.polytope, "b");
    print_polytope(B.polytope, "r");
    cout << "Done!" << endl;
}

#else
void print_data(Param* p, Drone A, Drone B)
{
    cout << "Printing data..." << endl;
    int numPoints = A.polytope.size();
    
    //cout << "printing data 1..." << endl;
    fprintf(p->out, "Pen b\n");
    
    for (int i = 0;i < numPoints; i ++)
        fprintf(p->out, "Line (%lf,%lf) (%lf,%lf)\n", A.polytope[i].x, A.polytope[i].y, A.polytope[(i+1)%numPoints].x, A.polytope[(i+1)%numPoints].y);
  
    numPoints = B.polytope.size();
  
    //cout <<numPoints << endl;

    //fprintf (out, "Ellipse (%lf,%lf) %lf %lf \n", plume.ovals[0].center.x, plume.ovals[0].center.y, R, R/2);
    //fprintf (out, "Ellipse (%lf,%lf) %lf %lf \n", plume.ovals[1].center.x, plume.ovals[1].center.y, R, R/2);

    //cout << "printing data 2..." << endl;
    fprintf (p->out, "Pen r\n");
    
    for (int i = 0;i < numPoints; i ++)
        fprintf (p->out, "Line (%lf,%lf) (%lf,%lf)\n", B.polytope[i].x, B.polytope[i].y, B.polytope[(i+1)%numPoints].x, B.polytope[(i+1)%numPoints].y);
 
    cout << "Done!" << endl;
    return ;
}
#endif

bool CrossPlume(Param* p, Drone &A, Drone &B, double alpha, PLUME &plume)
{
    Point start_pos = A.last;
   // if (abs(start_pos.x) < 2 && abs(start_pos.y) < 2)
     //   cout <<"start pos ... "<< start_pos.x << " " << start_pos.y <<" "<<check(A.position)<< endl;

    int crossing = A.inside;
    bool orient = true ;
    double alphainitial = p->alpha;
    bool endHere = false;
    
    int iterate = p->maxiterations;
    
    do{
        endHere = endHere || A.MoveDrone(p, p->alpha, p->epsilon * p->epsilon, plume, 0);
     //   B.MoveDrone(p, p->alpha, p->epsilon * p->epsilon, plume, 0);

        
        if (PointUtil::orientation(A.last, A.position, start_pos) == PointUtil::CLOCKWISE && p->alpha > 0)
            orient = false ;
        if (PointUtil::orientation(A.last, A.position, start_pos) == PointUtil::COUNTERCLOCKWISE && p->alpha < 0)
            orient = false ;
      //  if (crossing == A.inside)
        //    Sync(p, A,B,p->alpha, p->epsilon * p->epsilon, plume);
      //  else
        //    Sync(p, A,B,p->alphainitial, p->epsilon * p->epsilon, plume);
        
        if (p->alpha > 0)
            p->alpha += p->epsilon;
        else
            p->alpha -= p->epsilon;
        A.angleTurned += abs(alphainitial);
     //   cout << "testing cross plume ... "<< A.position.x << " " << A.position.y << endl;
     //   cout << "testing cross plume ... "<< B.position.x << " " << B.position.y << endl;
        iterate--;
        if (iterate < 0){
            print_data(p, A,B);
            fclose(p->out);
            exit(0);
        }
       // cout<<crossing<<" "<<A.inside<<" "<<A.droneIn<<endl;
    }while ( (crossing == A.inside) && orient && (!endHere) );
        
    if ( (crossing == A.inside) && (!endHere) ){
        A.polytope.pop_back();
        A.position = A.last;
        A.angleTurned -= abs(alphainitial);
        double dx = start_pos.getX() - A.position.getX();
        double dy = start_pos.getY() - A.position.getY();
        double gradient = atan2(dy, dx);
       
    //    A.position = A.last;
     //   B.position = B.last;
        
        Point d1 = start_pos - A.position;
        Point motion;
        if (d1.length() > (p->epsilon*p->epsilon) )
            motion = PointUtil::vector(gradient, p->epsilon * p->epsilon);
        else
            motion = PointUtil::vector(gradient, d1.length());
        motion = A.position + motion;
        Point d2 = start_pos - motion;
     //   cout<<A.position.x<<" " <<A.position.y<<" "<<d1.length()<< " distances " << d2.length()<<endl;

        if (d1.length() < d2.length()){
            reverse(gradient);
        }
        A.angleTurned += changeGradient(A.nabla + p->alpha, gradient);
        A.nabla = gradient;
     //   B.nabla = gradient;
     //   cout << start_pos.x << " starting here " << start_pos.y << " "<<gradient<<" "<<check(start_pos) <<" "<<check(A.position)<<" "<<A.inside<< endl;
     //   cout << "testing cross plume2 ... "<< A.position.x << " " << A.position.y << endl;
     //   cout << "testing cross plume2 ... "<< A.last.x << " " << A.last.y << endl;
        int iter = 0;
        while ( (crossing == A.inside) && (!endHere) ){
            d1 = start_pos - A.position;
            if (d1.length() > (p->epsilon * p->epsilon) )
            {
                endHere = endHere || A.MoveDrone(p, 0, p->epsilon*p->epsilon, plume, 0);
           //     Sync(p, A,B,alphainitial, p->epsilon * p->epsilon, plume);

            //    B.MoveDrone(p, 0, p->epsilon * p->epsilon, plume, 0);
             //   B.MoveDrone(p, 0, p->epsilon*p->epsilon, plume);
            }
            else{
                endHere = endHere || A.MoveDrone(p, 0, d1.length(), plume, 0);
            //    Sync(p, A,B,alphainitial, d1.length(), plume);

             //   B.MoveDrone(p, 0, d1.length(), plume, 0);

            //    B.MoveDrone(p, 0, d1.length()/2, plume);
            }
            
         //   if (abs(A.position.x) > 2 || abs(A.position.y) > 2)
           //     break ;
            ++iter;
            if (iter > p->maxiterations)
            {
                cout<<"CrossPlume() Iterations exceeding ..."<<endl;
                print_data(p, A,B);
                fclose(p->out);
                exit (0);
            }
        //    Sync2 (A,B, p->alpha);
            //cout << "testing cross plume2 ... "<< A.position.x << " " << A.position.y << endl;
      //      cout << "testing cross plume2 ... "<< B.position.x << " " << B.position.y << endl;
        }
      //  B.nabla = A.nabla;
     //   Sync(p, A,B,alphainitial, p->epsilon * p->epsilon, plume);
    }
    
    return endHere;
}


// FIX ME: Make this work for the critical path tracing
bool CrossCriticalPath(Param* p, Drone &A, Drone &B, double alpha, criticalPath &cp)
{
    cout << "Inside CrossCriticalPath()" << endl;

    Point start_pos    = A.last; 
    int   crossing     = A.side; 
    bool  orient       = true;
    double alphainitial = p->alpha;
    bool  endHere      = false;
    int   iterate      = p->maxiterations;

    // — remember what the last side was —
    int previousSide = 0;

    // —— initial “sketch” loop ——
    do {
        A.MoveDrone(p, p->alpha, p->epsilon, cp.diffepsilon, 0);

        CrossData crossData = cp.getCross(p, A, B, p->alpha, p->epsilon);
        if (crossData.second) {
            vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
            A.LearnGradient(p, p->alpha, p->epsilon, crossData.first, B, gradient_vec);
            B.LearnGradient(p, p->alpha, p->epsilon, crossData.first, A, gradient_vec);
        }

        endHere = endHere || cp.foundSource(A, B);

        // check orientation relative to the starting point
        Point last     = A.last;
        Point position = A.position;
        if ((PointUtil::orientation(last, position, start_pos) == PointUtil::CLOCKWISE) &&
            (p->alpha > 0))
        {
            orient = false;
        }
        if ((PointUtil::orientation(last, position, start_pos) == PointUtil::COUNTERCLOCKWISE) &&
            (p->alpha < 0))
        {
            orient = false;
        }

        // advance alpha
        if (p->alpha > 0) p->alpha += p->epsilon;
        else           p->alpha -= p->epsilon;

        A.angleTurned += std::abs(alphainitial);

        if (--iterate < 0) {

          break;
            printf("\n\ndying here \n\n");
            print_data(p, A, B);
            fclose(p->out);
            std::exit(0);
        }

    } while ((crossing == A.side) && orient && !endHere);

    // —— “walk-back” to critical point if we never left the original side ——
    if ((crossing == A.side) && !endHere) {
        // undo the last two steps
        A.polytope.pop_back();
        A.polytope.pop_back();
        A.position     = A.last;
        A.angleTurned -= std::abs(alphainitial);

        // point back towards the start position
        double dx       = start_pos.getX() - A.position.getX();
        double dy       = start_pos.getY() - A.position.getY();
        double gradient = std::atan2(dy, dx);

        // take a small step along that direction
        Point pos = A.position;
        Point d1  = start_pos - pos;
        Point motion;
        if (d1.length() > p->epsilon*p->epsilon)
            motion = PointUtil::vector(gradient, p->epsilon*p->epsilon);
        else
            motion = PointUtil::vector(gradient, d1.length());
        motion = pos + motion;

        Point d2 = start_pos - motion;
        if (d1.length() < d2.length()) {
            reverse(gradient);
        }

        A.angleTurned += changeGradient(A.nabla + p->alpha, gradient);
        A.nabla       = gradient;

        // —— final stepping loop until we cross sides or find the source ——
        int iter = 0;
        while ((crossing == A.side) && !endHere) {
            Point new_pos = A.position;
            d1 = start_pos - new_pos;

            if (d1.length() > (p->epsilon*p->epsilon)) {
                A.MoveDrone(p, 0, p->epsilon*p->epsilon, cp.diffepsilon, 0);
            } else {
                A.MoveDrone(p, 0, d1.length(), cp.diffepsilon, 0);
            }

            CrossData crossData = cp.getCross(p, A, B, p->alpha, p->epsilon);
            if (crossData.second) {
                vector<double> gradient_vec = cp.getGradientAtPoint(crossData.first);
                A.LearnGradient(p, p->alpha, p->epsilon, crossData.first, B, gradient_vec);
                B.LearnGradient(p, p->alpha, p->epsilon, crossData.first, A, gradient_vec);
            }

            // ——— only print when the side actually changes ———
            if (A.side != previousSide) {
                cout << "==== DRONE " << A.id << " SIDE NOW " << A.side << " ====" << endl;
                previousSide = A.side;
            }

            endHere = endHere || cp.foundSource(A, B);

            if (++iter > p->maxiterations) {
                cout << "CrossCriticalPath() Iterations exceeding ..." << endl;
                print_data(p, A, B);
                fclose(p->out);
                std::exit(0);
            }
        }
    }

    return endHere;
}





/***********************************************************************/
void sketch_algorithm(Param* p)
{
    // 1) Preparing Logging & Stats Tracking    
    //print_test_infrastructure_info();   //print initialization stats from logging.h
    Stats stats;                        //Per-epoch info
    int epoch = 0;
    cout << "In epoch..." << epoch << endl;

    // 2) Initialize Drones & stat tracking

    Drone A ('A',
            p->drone_start_A, 
            p->drone_start_A, 
            1, 
            0, 
            true);
    
    Drone B ('B',
            p->drone_start_B, 
            p->drone_start_B, 
            2, 
            0, 
            false);

    fprintf(p->out,"MoveDrone1 %c\n%f %f\n",A.id,p->drone_start_A.x,p->drone_start_A.y);
    fprintf(p->out,"MoveDrone1 %c\n%f %f\n",B.id,p->drone_start_B.x,p->drone_start_B.y);
    Point startPoint = {(p->drone_start_A.x + p->drone_start_B.x) / 2,
                        (p->drone_start_A.y + p->drone_start_B.y) / 2};

    double res = p->epsilon*p->epsilon; // how much resolution should we calculate contourlines at?
    criticalPath cp(p, startPoint, p->epsilon, res);
    

    bool loopEnd = false ;
    do{
        // --- each "epoch" is one pass through this inner loop ---
        int iter = 0;
        stats.reset();

        // record starting level
        stats.startLevel = cp.levelAt(p, A.position); 

        //CrossData crossData = cp.getCross(p, A, B, p->alpha, p->epsilon); //Grabbed outside for stats recording scope
        CrossData crossData{ Point(0,0), false};

#ifdef LEGACY
        while (((A.numCross + B.numCross == 0) || (3 == A.side + B.side)) 
                && (!loopEnd))
#endif//LEGACY
#ifndef LEGACY
        if(((A.numCross + B.numCross == 0) || (3 == A.side + B.side)) 
                && (!loopEnd))
#endif//LEGACY
        {

            // 3) Move both drones
            A.MoveDrone(p, p->alpha, p->epsilon, cp.diffepsilon, 1);
            B.MoveDrone(p, p->alpha, p->epsilon, cp.diffepsilon, 1);
            
            loopEnd = loopEnd || cp.foundSource(A, B);
            
            // 4) Check for a "cross" and if so, learn gradient. 
            CrossData crossData = cp.getCross(p, A, B, p->alpha, p->epsilon);
            if(crossData.second){
                auto gradient_vec = cp.getGradientAtPoint(crossData.first);
                A.LearnGradient(p, p->alpha, p->epsilon, crossData.first, B, gradient_vec);
                
                
                B.LearnGradient(p, p->alpha, p->epsilon, crossData.first, A, gradient_vec);
            }

            ++iter;
            if (iter > p->maxiterations) {
                // Fatal: Dump & Exit
                printf("\n\n OWWWWW\n\n");
                print_data(p, A,B); 
                fclose(p->out);
                exit (1);
            }
        }
            
        // 5) Record Statistics
        stats.endLevel    = cp.levelAt(p, A.position);        
        stats.gradX       = A.currentContourGradient[0];   
        stats.gradY       = A.currentContourGradient[1];  
        stats.angle       = A.nabla;
        stats.droneA      = { A.position.x,  A.position.y };
        stats.droneB      = { B.position.x,  B.position.y };
        stats.contourSize = cp.currentContourSize(p, A.position);
        stats.critPt      = { crossData.first.x, crossData.first.y };
        stats.tangentA    = {A.currentTangent[0], A.currentTangent[1]};
        stats.tangentB    = {B.currentTangent[0], B.currentTangent[1]};
        auto cpPoints = cp.getCriticalPathPoints();
        auto lastCP  = cpPoints.back();
        stats.computeCross(-lastCP.y, lastCP.x);

            
        stats.print(epoch++);

        cout << "AFTER STATS PRINT" << endl;

        if ( (A.side + B.side) != 3)
        {
            cout << "A.side + B.side = " << (A.side + B.side) << endl;
            // If A crosses
            if ( (A.side == 2) && (B.side == 2) )
            {
                cout << "A crossed into B" << endl;
                p->alpha = -p->epsilon;
                loopEnd = loopEnd || CrossCriticalPath(p, B, A, p->alpha, cp);
                cout << "Syncing..." << endl;
                Sync(p, B, A, p->alpha, p->epsilon, cp); // FIX ME
                A.nabla = B.nabla;
            }
            else // B crosses
            {
                cout << "B crossed into A" << endl;
                p->alpha = p->epsilon;
                loopEnd = loopEnd || CrossCriticalPath(p, A, B, p->alpha, cp);
                cout << "Syncing..." << endl;
                Sync(p, A, B, p->alpha, p->epsilon, cp);
                B.nabla = A.nabla;
            }
        }

        
    }while (!loopEnd);
 
    printSummary(p, A);
    print_data (p, A,B);

    printf("Thanks for coming to my TED talk  \n\n");
    return ;
}   


void test_infrastructure(Param* p)
{
    p->gaussianCenter.clear();
    p->gaussianVar.clear();
    //initial direction in radians
    p->alpha = PI/2; // slightly off pi/2 to try to cause early crossing
    //velocity
    p->epsilon = 0.1;

    p->CROSSBOUND = 100;
    p->majorAxis = 0.25;
    p->minorAxis = 0.75;
    
    p->DIST = sqrt(49);
    
    p->THRESHOLD = exp(-p->majorAxis*p->majorAxis);
   
    p->drone_start_B = Point(1 + p->DIST*p->epsilon*0.5,-2.38);
    p->drone_start_A = Point(1 - p->DIST*p->epsilon*0.5,-2.38);
    p->drone_start_BB = Point(1 + p->DIST*p->epsilon*0.5,-2.38);
    p->drone_start_AB = Point(1 - p->DIST*p->epsilon*0.5,-2.38);
   
#ifndef LEGACY
    p->maxiterations = 10;
    p->num = 1;
    p->gaussianCenter.push_back(Point(1,1));
    p->gaussianVar.push_back(Point(1,3));
#endif //LEGACY
#ifdef LEGACY
    // data/gaussian_contours_data_7.txt  
    p->num = 4;
    p->gaussianCenter.push_back(Point(1,1));
    p->gaussianCenter.push_back(Point(-2,0));
    p->gaussianCenter.push_back(Point(1,0));
    p->gaussianCenter.push_back(Point(4,0));
    
    p->gaussianVar.push_back(Point(0.6,0.6));
    p->gaussianVar.push_back(Point(0.8,1.0));
    p->gaussianVar.push_back(Point(0.9,1.8));
    p->gaussianVar.push_back(Point(0.5,0.5));
#endif //LEGACY
    print_test_infrastructure_info(p); //print initialization stats
        
}

void test_infv2(Param* p){

    p->gaussianCenter.clear();
    p->gaussianVar.clear();
    //initial direction in radians
    p->alpha = 1.47; // slightly off pi/2 to try to cause early crossing
    //velocity
    p->epsilon = 0.1;

    p->CROSSBOUND = 100;
    p->majorAxis = 0.25;
    p->minorAxis = 0.25;
    
    p->DIST = sqrt(49);
    
    p->THRESHOLD = exp(-p->majorAxis*p->majorAxis);
   
    p->drone_start_B = Point(1 + p->DIST*p->epsilon*0.5,-2.38);
    p->drone_start_A = Point(1 - p->DIST*p->epsilon*0.5,-2.38);

    p->drone_start_BB = Point(1 + p->DIST*p->epsilon*0.5,-2.38);
    p->drone_start_AB = Point(1 - p->DIST*p->epsilon*0.5,-2.38);
   

    p->maxiterations = 100;
    p->num = 1;
    p->gaussianCenter.push_back(Point(1,1));
    p->gaussianVar.push_back(Point(1,1));

}
