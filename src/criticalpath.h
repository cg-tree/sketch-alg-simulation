#ifndef CRITICALPATH_H
#define CRITICALPATH_H

struct criticalPath{
    Point stcriticalPoint;
    double diffepsilon;
    vector<Point> criticalPathPoints;
    double contourRes;
    vector<double> lastCPCurvatures;
    static const int LEFT = 1;
    static const int RIGHT = 2;
    
    /* begin modified def*/
    // get derivaties of curvature by fitting spline
    CubicSpline curv_spline;

    vector<double> xValues;

    vector<Point> localContour;
    /* end modified def*/

    criticalPath(Param* p, Point start, double epsilon, double res)
    : stcriticalPoint(start), diffepsilon(p->epsilon), contourRes(res) {
        // Initialize the critical path by finding the first critical point
        double level = getGaussian(p, start);
        vector<Point> initialContour = getGaussianContours(p,
                                       level,
                                       contourRes,
                                       start.x - p->DIST * p->epsilon, 
                                       start.x + p->DIST * p->epsilon, 
                                       start.y - p->DIST * p->epsilon, 
                                       start.y + p->DIST * p->epsilon);
        if (!initialContour.empty()) {
            Point firstCriticalPoint = getCriticalPoint(p, initialContour);
            criticalPathPoints.push_back(firstCriticalPoint);
        } else {
            std::cerr << "Failed to initialize critical path: no contour points found." << std::endl;
        }
    }

    double levelAt(Param* p, const Point &pt) const {
        return getGaussian(p, pt);
    }

    /// Compute (and store) the contour around point p and return its number of points
    int currentContourSize(Param* p, const Point &pt) {
        // 1) Evaluate the field at p
        double lvl = getGaussian(p, pt);

        // 2) Rebuild the contour at that level, within ±p->DIST*diffepsilon of p
        localContour = getGaussianContours(p,
            lvl,
            contourRes,
            pt.x - p->DIST * diffepsilon,
            pt.x + p->DIST * diffepsilon,
            pt.y - p->DIST * diffepsilon,
            pt.y + p->DIST * diffepsilon
        );

        // 3) Return how many sample‐points we got
        return static_cast<int>(localContour.size());
    }

#ifdef LEGACY
    vector<Point> getCriticalPathPoints() {
        std::sort(criticalPathPoints.begin(), criticalPathPoints.end()); 
        vector<Point> cleaned = removeDuplicatePoints(criticalPathPoints, diffepsilon);

        return cleaned;
    }
#endif //LEGACY

#ifndef LEGACY
    //removed the sorting becase the operator is not distance
    //and the last critical point isn't guaranteed to remain at the end
    vector<Point> getCriticalPathPoints() {
        vector<Point> cleaned = removeDuplicatePoints(criticalPathPoints,diffepsilon);
        return cleaned;
    }

#endif //LEGACY
    std::pair<CubicSpline, CubicSpline> getCriticalPathSpline() {
        // just want to sort just in case
        vector<Point> criticalPathPoints = getCriticalPathPoints();

        vector<double> xValues;
        vector<double> yValues;
        vector<double> t;
        double totalLength = 0.0;
        t.push_back(0.0);
        xValues.push_back(criticalPathPoints[0].x);
        yValues.push_back(criticalPathPoints[0].y);
        for (size_t i = 1; i < criticalPathPoints.size(); ++i) {
            double dx = criticalPathPoints[i].x - criticalPathPoints[i-1].x;
            double dy = criticalPathPoints[i].y - criticalPathPoints[i-1].y;
            totalLength += std::hypot(dx, dy);
            t.push_back(totalLength);
            xValues.push_back(criticalPathPoints[i].x);
            yValues.push_back(criticalPathPoints[i].y);
        }
        
        CubicSpline cp_spline_x;
        cp_spline_x.setPoints(t, xValues);

        CubicSpline cp_spline_y;
        cp_spline_y.setPoints(t, yValues);


        return std::make_pair(cp_spline_x, cp_spline_y);
    }

    vector<double> getGradientAtPoint(Point P){

        std::pair<CubicSpline, CubicSpline> cp_spline = getCriticalPathSpline();
        CubicSpline cp_spline_x = cp_spline.first;
        CubicSpline cp_spline_y = cp_spline.second;
    

        double tMin = cp_spline_x.x.front();
        double tMax = cp_spline_x.x.back();
        double bestT = tMin;
        double minDist = std::numeric_limits<double>::max();
    
        for (double t = tMin; t <= tMax; t += contourRes) { 
            double xt = cp_spline_x(t);
            double yt = cp_spline_y(t);
            double dist = std::hypot(xt - P.x, yt - P.y);
            if (dist < minDist) {
                minDist = dist;
                bestT = t;
            }
        }
    
        double dx = cp_spline_x.getDerivative(bestT);
        double dy = cp_spline_y.getDerivative(bestT);

        return {dx, dy};


    }

  
    // This function is used to get the tangent vector of the contour
    // that the drone is currently at. We need both positions because
    // we want the tangent vector to be oriented in the direction of the
    // critical point.
    vector<double> getContourTangent(Param* p, Point A, Point B){
        Point current = A;
        vector<Point> surroundingPoints;
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0) continue; 
            if (i != 0 && j != 0) { 
                Point offset = Point(i * diffepsilon, j * diffepsilon);
                surroundingPoints.push_back(current + offset);
            }
            }
        }

        vector<double> gradient = concentration_gradient_LSQ(p, surroundingPoints, current);

        double centerX = (A.x + B.x) * 0.5;
        double centerY = (A.y + B.y) * 0.5;
        double level = getGaussian(p, current);
        //cout << "level: " << level << endl;
        //cout << "contour res:" <<  contourRes << endl;
        vector<Point> localContour = getGaussianContours(p, level, 
            contourRes, 
            centerX - 1*p->DIST*p->epsilon, // multiplier here may change based
            centerX + 1*p->DIST*p->epsilon, // on assumption 1.
            centerY - 1*p->DIST*p->epsilon, 
            centerY + 1*p->DIST*p->epsilon); 

        //cout << "size of local contour: " << localContour.size() << endl;

        Point criticalPoint = getCriticalPoint(p, localContour);
        //cout << "critical point: " << criticalPoint.x << " " << criticalPoint.y << endl;

            vector<double> r = {criticalPoint.x - current.x, criticalPoint.y - current.y};
            vector<double> t = {-gradient[1], gradient[0]};

            double r_norm = sqrt(r[0] * r[0] + r[1] * r[1]);
            if (r_norm > 1e-9) {
                r[0] /= r_norm;
                r[1] /= r_norm;
            }

            double t_norm = sqrt(t[0] * t[0] + t[1] * t[1]);
            if (t_norm > 1e-9) {
                t[0] /= t_norm;
                t[1] /= t_norm;
            }


        double dotProduct = r[0] * t[0] + r[1] * t[1];
        if(dotProduct < 0){
            t[0] = -t[0];
            t[1] = -t[1];
            //return t;
        }
        /*else{
            return t;
        }*/
        fprintf(p->out,"Tangent %f,%f\n%f %f\n",A.x,A.y,t[0],t[1]); 
        return t;
    }


    std::pair<int,int> checkCross(Param* p, Drone& droneA, Drone& droneB, Point motion)
    {
        // Need to get the gradient around the initial point
        // and the end point in order to compare sign of 
        // the gradient
 
#ifdef LEGACY
        //get tangent vector, which is normal to the gradient vector
        vector<double> tangentA = getContourTangent(p, droneA.position + motion, droneB.position + motion);
        vector<double> tangentB = getContourTangent(p, droneB.position + motion, droneA.position + motion);
#endif//LEGACY

#ifndef LEGACY
        //we don't have foresight... we can't use information we don't have
        vector<double> tangentA = getContourTangent(p, droneA.position, droneB.position);
        vector<double> tangentB = getContourTangent(p, droneB.position, droneA.position);
#endif//LEGACY

        droneA.currentTangent = tangentA;
        droneB.currentTangent  = tangentB;

#ifdef LEGACY
        droneA.currentContourGradient = {-tangentA[1], tangentA[0]};
        droneB.currentContourGradient = {tangentB[1], -tangentB[0]};
#endif //LEGACY
#ifndef LEGACY
        Point ortha = get_orthogonal_vector(tangentA[0],tangentA[1]);
        droneA.currentContourGradient = {ortha.x,ortha.y};
        Point orthb = get_orthogonal_vector(tangentB[0],tangentB[1]);
        droneB.currentContourGradient = {orthb.x,orthb.y};

#endif //LEGACY
        

        vector<Point> cpPoints = getCriticalPathPoints();
        vector<double> vectorBetween = {-0.5*(ortha.x + orthb.x), -0.5*(ortha.y+orthb.y)};
        Point lastCPPoint;
        lastCPPoint.x = 0;
        lastCPPoint.y = 0;

        if (!cpPoints.empty()) {
            lastCPPoint = cpPoints.back();

            // FIX ME: supposed to be vector normal to critical point,
            // but not sure how robust this is.
#ifdef LEGACY
            vectorBetween = {-lastCPPoint.y, lastCPPoint.x}; 
#endif //LEGACY
#ifndef LEGACY
            Point vecbet = get_orthogonal_vector(vectorBetween[0],vectorBetween[1]);
            //vectorBetween = {vecbet.x, vecbet.y};
#endif //LEGACY
        }

        fprintf(p->out,"VectorBetween %f,%f\n%f %f\n",lastCPPoint.x,lastCPPoint.y,vectorBetween[0],vectorBetween[1]);

        double crossProductA = vectorBetween[0] * tangentA[1] - vectorBetween[1] * tangentA[0];
        double crossProductB = vectorBetween[0] * tangentB[1] - vectorBetween[1] * tangentB[0];


        //cout << "cross prod A: " << crossProductA << endl;
        //cout << "cross prod B: " << crossProductB << endl;

        
        std::pair<int, int> side;
        if (crossProductA > contourRes){
            side.first = RIGHT;
        }else{
            side.first = LEFT;
        }

        if (crossProductB > contourRes){
            side.second = RIGHT;
        }else{
            side.second = LEFT;
        }

        return side;


    }

    Point getCriticalPoint(Param* p, vector<Point>& localContour)
    {
        lastCPCurvatures.clear();

        for(auto& point : localContour){
            vector<Point> surroundingPoints;
            for(int i = -1; i < 2; i++){
                for(int j = -1; j < 2; j++){
                    if (i == 0 && j == 0) continue; 
                    Point offset = Point(i*diffepsilon, j*diffepsilon);
                    surroundingPoints.push_back(point + offset);
                }
            }

            double curvature = calc_curvature_LSQ(p, surroundingPoints, point); 
            lastCPCurvatures.push_back(curvature);
        }


        // get derivaties of curvature by fitting spline
        CubicSpline curv_spline;
        
        // need to get x values vs curvature
        vector<double> xValues;
        for(auto& point : localContour){
            xValues.push_back(point.x);
        }
        curv_spline.setPoints(xValues, lastCPCurvatures);
        vector<double> curv_splineDerivatives;
        for (size_t i = 0; i < xValues.size(); ++i) {
            double derivative = curv_spline.getDerivative(xValues[i]);
            curv_splineDerivatives.push_back(std::pow(derivative, 2));
        }

        // now get where minimized since we squared derivative
        size_t bestIdx = 0;
        double minSqVal = std::numeric_limits<double>::max();
        for (size_t i = 1; i + 1 < curv_splineDerivatives.size(); ++i) {

            if (curv_splineDerivatives[i] < minSqVal) {
                    bestIdx = i;
                    minSqVal = curv_splineDerivatives[i];
                }
        } 


        Point criticalPoint = localContour[bestIdx];

        criticalPathPoints.push_back(criticalPoint);
        
        fprintf(p->out,
            "CriticalPoint %d\n%f %f\n",
            criticalPathPoints.size(),
            criticalPoint.x,
            criticalPoint.y);
        //curvature_gradient_LSQ(p,surroundingPoints,point);

        return criticalPoint;
    }

    Point getCrossingPoint(Param* p, Drone droneA, Drone droneB){
        // Note that Drone A is the drone we are determining
        // the crossing point for 
        double startLevel = getGaussian(p, droneA.position);
        double endLevel = getGaussian(p, droneA.last);
        cout << "getCrossingPoint() |  start level: " << startLevel << "   end level: " << endLevel << endl;

        // used to look for point that minimizes the curvature
        // (critical point numerically)
        auto findMinIndex = [](const std::vector<double>& v) -> size_t {
            return std::distance(v.begin(), std::min_element(v.begin(), v.end()));
        };

        double startCenterX = (droneA.position.x + droneB.position.x) * 0.5;
        double startCenterY = (droneA.position.y + droneB.position.y) * 0.5;

        // FIX ME: is this the right way to deal with these two cases?

        // get contour lines within box around start and end
        if (std::abs(startLevel - endLevel) > diffepsilon){
            // we consider this to be two different contours
            // and will need to evaluate the critical path 
            // between two crtitical points

            // FIX ME: Need to determine size of region that we will get
            // contours for.

            double endCenterX = (droneA.last.x + droneB.last.x) * 0.5;
            double endCenterY = (droneA.last.y + droneB.last.y) * 0.5;
            
            vector<Point> localContourStart = getGaussianContours(p,
                startLevel, 
                contourRes, 
                startCenterX - 1*p->DIST*p->epsilon, 
                startCenterX + 1*p->DIST*p->epsilon, 
                startCenterY - 1*p->DIST*p->epsilon, 
                startCenterY + 1*p->DIST*p->epsilon); 

            vector<Point> localContourEnd = getGaussianContours(p,
                endLevel,
                contourRes,
                endCenterX - 1*p->DIST*p->epsilon,
                endCenterX + 1*p->DIST*p->epsilon,
                endCenterY - 1*p->DIST*p->epsilon,
                endCenterY + 1*p->DIST*p->epsilon);
         
            // critical path is path between two critical points                                                      
            Point criticalPointStart = getCriticalPoint(p, localContourStart);
            Point criticalPointEnd = getCriticalPoint(p, localContourEnd);


            
            criticalPathPoints.push_back(criticalPointStart);
            criticalPathPoints.push_back(criticalPointEnd);

            // find intersection between critical path and start and end
            double startX1 = droneA.position.getX();
            double startY1 = droneA.position.getY();
            double EndX2 = droneA.last.getX();
            double EndY2 = droneA.last.getY();
            double cpX3 = criticalPointStart.getX();
            double cpY3 = criticalPointStart.getY();
            double cpX4 = criticalPointEnd.getX();
            double cpY4 = criticalPointEnd.getY();

            double denom = (startX1 - EndX2)*(cpY3 - cpY4) - (startY1 - EndY2)*(cpX3 - cpX4);
            if (denom == 0){
                cout << "Exception! No intersection!"<<endl;
                exit(0);
            }


            double px = ((startX1 * EndY2 - startY1 * EndX2) * (cpX3 - cpX4) - (startX1 - EndX2) * (cpX3 * cpY4 - cpY3 * cpX4)) / denom;
            double py = ((startX1 * EndY2 - startY1 * EndX2) * (cpY3 - cpY4) - (startY1 - EndY2) * (cpX3 * cpY4 - cpY3 * cpX4)) / denom;
            Point intersectionPoint(px, py);

            return intersectionPoint;
                

        }else{
            // we consider this to be a single contour
            // and will need to evaluate where the critical point is
            vector<Point> localContour = getGaussianContours(p,
                startLevel, 
                0.01, 
                startCenterX - 1*p->DIST*p->epsilon,
                startCenterX + 1*p->DIST*p->epsilon,
                startCenterY - 1*p->DIST*p->epsilon,  
                startCenterY + 1*p->DIST*p->epsilon);

            Point criticalPoint = getCriticalPoint(p, localContour);
            criticalPathPoints.push_back(criticalPoint);
            return criticalPoint;

        }
        
    }
    
    CrossData getCross(Param* p, Drone& droneA, Drone& droneB, double alpha, double dist)
    {
    
       Point motion = PointUtil::vector(droneA.nabla + p->alpha, dist/100);
       cout << "Motion:  X = " << motion.x << "   Y = " << motion.y << "      ";
       
       for(int i = 0; i < p->maxiterations; i++){

            std::pair<int,int> crossInfo = checkCross(p, droneA, droneB, motion);
    
            //cout << "cross info" << crossInfo.first << ", " << crossInfo.second << endl;  

            Point crossPoint = Point(0,0);
            if(crossInfo.first != LEFT) // location of drone A
            {
                
                crossPoint = getCrossingPoint(p, droneA, droneB);
                droneA.numCross++;
                droneA.side = RIGHT;

                return CrossData(crossPoint, 1);

            }else if(crossInfo.second != RIGHT){
                
                
                crossPoint = getCrossingPoint(p, droneB, droneA);
                droneB.numCross++;
                droneB.side = LEFT;
                   
                return CrossData(crossPoint, 2);
            }
#ifndef LEGACY
         return CrossData( Point (0,0), 0 );

         double startCenterX = (droneA.position.x + droneB.position.x) * 0.5;
         double startCenterY = (droneA.position.y + droneB.position.y) * 0.5;
         return CrossData( Point (startCenterX,startCenterY), 0 );
            //this is where the drone is being moved without proper authorization
            //drone movement is not the responsibility of this function
#endif//LEGACY
            motion += motion;
       }
       
       return CrossData( Point (0,0), 0 );
   }

   // FIX ME: We know that a drone has encountered a source when 
   // assumptions are violated. Is there a better way to do this?
   bool foundSource(Drone droneA, Drone droneB){
        // FIX ME: if drone has encountered a concave contour we 
        // know its a source

        // FIX ME: need to check if still using this
        if (droneA.lastTangent.size() == 0){
            return false;
        }

        double crossProdLast = droneA.lastTangent[0] * droneB.lastTangent[1] - 
                               droneA.lastTangent[1] * droneB.lastTangent[0];

        double crossProdCurrent = droneA.currentTangent[0] * droneB.currentTangent[1] - 
                               droneA.currentTangent[1] * droneB.currentTangent[0];
        
        // check if the cross product of the last and current gradient
        // is negative, which indicates that the contours have
        // changed from concave to convex or vice versa
        if (crossProdLast * crossProdCurrent < 0){
            // double check if the cross product is negative
            // and positive where expected?
            cout << "found source" << endl;
            return true;
        }
        return false;


   }
};

#endif //CRITICALPATH_H
