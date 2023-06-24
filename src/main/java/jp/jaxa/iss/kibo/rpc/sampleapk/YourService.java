package jp.jaxa.iss.kibo.rpc.sampleapk;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import android.util.Log;



import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

// OpenCV library
import org.opencv.core.Mat;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    double[][] target = {{0,0,0},//NULL
                        {11.2325, -10.15, 5.4725},//tar1
                        {10.615384, -9.115172, 4.46203},//tar2
                        {10.71, -7.765, 4.42},//tar3
                        {10.43, -6.615, 5.18531},//tar4
                        {11.202, -7.951, 5.53},//tar5
                        {11.502, -9.067, 4.935},//tar6
                        {11,-9.4,5.1},//pointA
                        {10.81,-7.37,4.9},//pointB
                        {11.381944,-8.566172,3.76203},//QR
                        {9.815, -9.806, 4.293},//start
                        {11.143,-6.7607,4.9654}//Goal
    };

    float[][] quater =  {{0,0,0,0},//NULL
                        {0f, 0f, 0.707f, -0.707f},//tar1
                        {0f, 0.707f, 0f, 0.707f},//tar2
                        {0f, 0.707f, 0f, 0.707f},//tar3
                        {0f, 0f, -1f, 0f},//tar4
                        {0.707f, 0f, 0.707f, 0f},//tar5
                        {0f, 0f, 0f, 1f},//tar6
                        {0f, 0f, 0f, 1f},//pointA
                        {0f, 0f, 0f, 1f},//pointB
                        {0f, 0.707f, 0f, 0.707f},//QR
                        {0f, 0f, 0f, 1f},//start
                        {0f, 0f, 0f, 1f}//Goal
    };

    double[][] Target = {{11.2325, -10.15, 5.4725},
                        {10.615384, -9.115172, 4.46203},
                        {10.71, -7.765, 4.42},
                        {10.43, -6.615, 5.18531},
                        {11.202, -7.951, 5.53},
                        {11.502, -9.067, 4.935},

                        {11,-9.4,5.1},{10.81,-7.37,4.9}
    };


    //constant Quaternion
    float[][] QUA =     {{0f, 0f, 0.707f, -0.707f},
                        {0f, 0.707f, 0f, 0.707f},
                        {0f, 0.707f, 0f, 0.707f},
                        {0f, 0f, -1f, 0f},
                        {0.707f, 0f, 0.707f, 0f},
                        {0f, 0f, 0f, 1f}};

    double[] Start = {9.815, -9.806, 4.293};

    //x มากไปขวา y มากยิ่งไกล z มากยิ่งต่ำ
    //Group CheckPoint
    double[] P_GroupA = {11,-9.4,5.1};//11.115,-9.2821,4.8101
    double[] P_GroupB = {10.81,-7.37,4.9};//10.61,-7.57,5
    int cnt_checkpoint = 1;

    //default Quaternion
    Quaternion Qua_reseT = new Quaternion(0f, 0.707f, 0f, 0.707f);
    float[] Qua_reset = {0f, 0f, 0f, 1f};
    //Current Point
    double Curr[] = Start;
    double Curr2[] = Start;
    int current_key = 10; // at start

    double[] Goal = {11.143,-6.7607,4.9654};
    Quaternion Qua_Goal = new Quaternion(0f, 0f, -0.707f, 0.707f);
    double[] QR = {11.381944,-8.566172,3.76203};
    Quaternion Qua_QR = new Quaternion(0f, 0.707f, 0f, 0.707f);


    double[] point = {10.4, -10.1, 4.47};
    Quaternion quaternion = new Quaternion(0f, 0f, 0f, 1f);

    //Compare Value
    int temp = 0;
    int Shooting_count = 0;

    @Override
    protected void runPlan1(){
        // write your plan 2 here
        api.startMission();
        solution();


        //For QR Scan

        // turn on the front flash light
        api.flashlightControlFront(0.05f);

        // get QR code content
        String mQrContent = yourMethod();

        // turn off the front flash light
        api.flashlightControlFront(0.00f);


        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();

        Log.i("Goallll", "Move to ending");
        if(Curr == target[1] || Curr == target[2]){
            moving2(P_GroupA,Qua_reset);
        }
        moving(Goal, Qua_Goal);

        // send mission completion
        api.reportMissionCompletion(mQrContent);
    }


    @Override
    protected void runPlan2(){
        // the mission starts
        api.startMission();

        //move to first group point
        //moving2(P_GroupB,Qua_reset);
        //moving2(P_GroupA,Qua_reset);
        int loop_counter = 0;
        while (true){
            // get the list of active target id
            // move to a point

            List<Integer> list = api.getActiveTargets();
            int n = list.size();
            Log.i("klekle", "n = " + n);
            for(int i=0;i<n;i++){
                Log.i("klekle", "kle index " + i + " = " + list.get(i));
            }
            int key = 0,copykey = 0;
            for (int i = 0; i < n; i++) {
                Log.i("The Active Target is ", "" + list.get(0));
                if(n > 1 && i == 0){
                    key = find_min(list, n);
                    copykey = key;
                }
                else{
                    key = list.get(0);
                    if(key == copykey) {
                        key = list.get(1);
                    }
                }
                Checkpoint(key);
                /*
                Log.i("Remove", "index Remove = " + Integer.valueOf(key));
                list.remove(Integer.valueOf(key));
                */
            }

            /*
            // get a camera image
            Mat image = api.getMatNavCam();
            // irradiate the laser
            api.laserControl(true);
            // take active target snapshots
            int target_id = 1;
            api.takeTargetSnapshot(target_id);*/

            /* ************************************************ */
            /* write your own code and repair the ammonia leak! */
            /* ************************************************ */

            // get remaining active time and mission time
            List<Long> timeRemaining = api.getTimeRemaining();
            // check the remaining milliseconds of mission time
            if (timeRemaining.get(1) < 60000){
                break;
            }
            //if(n == 0)break;
            if (loop_counter == 3){
                break;
            }
            loop_counter++;
        }
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here

    }

    // You can add your method
    private String yourMethod(){
        String contents = null;
        byte count = 0;

        return "your method";
    }
    private void Shooting_point(int key){
        moving2(Target[key-1],QUA[key-1]);
        Log.i("SHOOTING!!!", "Moving to target : " + key);
        api.laserControl(true);
        Log.i("Snapshot", "target snapshot id : " + key);
        Shooting_count++;
        api.takeTargetSnapshot(key);
        //api.laserControl(false);
    }

    private void Checkpoint(int key){

        //Target 1
        if(key == 1 && Curr == Target[3]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 1 && Curr == Target[4]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 1 && Curr == Target[5]){
            Shooting_point(key);
            Curr = Target[key-1];
        }

        //Target 2
        else if(key == 2 && Curr == Start){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 2 && Curr == Target[3]){
            moving2(Target[4],Qua_reset);
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 2 && Curr == Target[4]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 2 && Curr == Target[5]){
            Shooting_point(key);
            Curr = Target[key-1];
        }

        //Target 3
        else if(key == 3 && Curr == Target[3]){
            Shooting_point(key);
            Curr = Target[key-1];
        }


        //Target 4
        else if(key == 4 && Curr == Target[0]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 4 && Curr == Target[2]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 4 && Curr == Target[4]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 4 && Curr == Target[5]){//
            Shooting_point(key);
            Curr = Target[key-1];
        }

        //Target 5
        else if(key == 5 && Curr == Target[0]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 5 && Curr == Target[1]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 5 && Curr == Target[3]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 5 && Curr == Target[5]){
            Shooting_point(key);
            Curr = Target[key-1];
        }

        //Target 6
        else if(key == 6 && Curr == Start){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 6 && Curr == Target[0]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 6 && Curr == Target[1]){
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 6 && Curr == Target[2]){
            moving2(P_GroupB,Qua_reset);
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 6 && Curr == Target[3]){//
            Shooting_point(key);
            Curr = Target[key-1];
        }
        else if(key == 6 && Curr == Target[4]){
            Shooting_point(key);
            Curr = Target[key-1];
        }

        //From PointA
        else if(key == 2 && Curr == Target[6]){
            Shooting_point(key);
            Curr = Target[key-1];
        }


        else if(key == 1 || key == 2 || key == 6 || key == 5) {
            if (Curr != P_GroupA && temp == 0) {//อาจจะเปลี่ยนไปเป็นเก็บตัวแปรอื่นเพื่อเทียบแทน ถ้าใช้เวลานานเกิน
                Log.i("Group_point", "Move to Group point : A");
                moving2(P_GroupA, Qua_reset);
                Curr2 = P_GroupA;/*
                for(int j = 0;j<3;j++){
                    Log.i("Current", "Current Point = " + Curr[j]);
                }*/
                temp = 1;
            }
            else if(temp == 1){
                moving2(Curr2,Qua_reset);
                temp = 0;
            }
            Log.i("Checkpoint", "checkpoint : " + cnt_checkpoint);
            Curr = Target[key-1];
            Shooting_point(key);
            //moving2(P_GroupA,Qua_reset);
        }
        else {
            if(Curr != P_GroupB && temp == 0){//อาจจะเปลี่ยนไปเป็นเก็บตัวแปรอื่นเพื่อเทียบแทน ถ้าใช้เวลานานเกิน
                Log.i("Group_point", "Move to Group point : B");
                moving2(P_GroupB,Qua_reset);
                Curr2 = P_GroupB;/*
                for(int j = 0;j<3;j++){
                    Log.i("Current", "Current Point = " + Curr[j]);
                }*/
                temp = 1;
            }
            else if(temp == 1){
                moving2(Curr2,Qua_reset);
                temp = 0;
            }
            Log.i("Checkpoint", "checkpoint : " + cnt_checkpoint);
            Shooting_point(key);
            Curr = Target[key-1];
            //moving2(P_GroupB,Qua_reset);
        }

        cnt_checkpoint++;
    }

    private int find_min(List<Integer> list, int n){
        double MAX = 1500.001;
        int key = 0;
        for(int i = 0;i<n;i++) {
            int temp = list.get(i) - 1;
            double distant = Math.sqrt(Math.pow(Curr[0] - Target[temp][0], 2) + Math.pow(Curr[1] - Target[temp][1], 2) + Math.pow(Curr[2] - Target[temp][2], 2));
            //find min distant Point
            if (distant < MAX) {
                MAX = distant;
                key = temp + 1;
            }
        }
        Log.i("Min Value", "Min target is = " + key);
        return key;
    }

    private void solution(){

        // Node
        int v = 12;

        // Adjacency list for storing which vertices are connected
        ArrayList<ArrayList<Integer>> adj = new ArrayList<ArrayList<Integer>>(v);
        for (int i = 0; i < v; i++) {
            adj.add(new ArrayList<Integer>());
        }

        //| 1 - 6 = target | 7 = A | 8 = B | 9 = QR | 10 = start | 11 = end |

        //Tar1
        add_edge(adj, 1, 4);
        add_edge(adj, 1, 5);
        add_edge(adj, 1, 6);
        add_edge(adj, 1, 7);
        add_edge(adj, 1, 8);
        //tar2
        add_edge(adj, 2, 5);
        //add_edge(adj, 2, 6);
        add_edge(adj, 2, 7);
        //tar3
        //add_edge(adj, 3, 4);
        add_edge(adj, 3, 8);

        //tar4
        add_edge(adj, 4, 5);
        add_edge(adj, 4, 6);
        add_edge(adj, 4, 8);
        //tar5
        add_edge(adj, 5, 6);
        add_edge(adj, 5, 7);
        add_edge(adj, 5, 8);
        //tar6
        add_edge(adj, 6, 7);
        add_edge(adj, 6, 8);
        //PointA -> PointB
        add_edge(adj, 7, 8);
        //QR
        add_edge(adj, 9, 6);
        add_edge(adj, 9, 7);
        add_edge(adj, 9, 10);
        //start
        add_edge(adj, 10, 2);
        add_edge(adj, 10, 6);
        add_edge(adj, 10, 7);
        //Goal
        add_edge(adj, 11, 3);
        add_edge(adj, 11, 4);
        add_edge(adj, 11, 5);
        add_edge(adj, 11, 7);
        add_edge(adj, 11, 8);

        //---------------start--------------\\

        //start == Curr, end == key,v == node

        int loop_counter = 0;
        while (true){
            // get the list of active target id
            // move to a point

            List<Integer> list = api.getActiveTargets();
            int n = list.size();
            Log.i("klekle", "n = " + n);
            for(int i=0;i<n;i++){
                Log.i("klekle", "kle index " + i + " = " + list.get(i));
            }
            int key = 0,copykey = 0;
            for (int i = 0; i < n; i++) {
                //Cut Function
                if(Shooting_count == 7){
                    Log.i("Target Break ", "7 targets have been shot");
                    break;
                }
                Log.i("The Active Target is ", "" + list.get(0));
                if(n > 1 && i == 0){
                    key = find_min2(list, n);
                    copykey = key;
                }
                else{
                    key = list.get(0);
                    if(key == copykey) {
                        key = list.get(1);
                    }
                }
                findShortestDistance(adj, current_key, key, v);


            }


            // get remaining active time and mission time
            List<Long> timeRemaining = api.getTimeRemaining();
            // check the remaining milliseconds of mission time
            if (timeRemaining.get(1) < 60000){
                break;
            }
            //if(n == 0)break;
            if (loop_counter == 4 || Shooting_count == 7){
                break;
            }
            loop_counter++;
        }

    }

    private void add_edge(ArrayList<ArrayList<Integer>> adj, int i, int j)
    {
        adj.get(i).add(j);
        adj.get(j).add(i);
    }

    // function to print the shortest distance and path
    // between source vertex and destination vertex
    private void findShortestDistance(
            ArrayList<ArrayList<Integer>> adj,
            int s, int dest, int v)
    {
        // predecessor[i] array stores predecessor of
        // i and distance array stores distance of i
        // from s
        int pred[] = new int[v];
        int dist[] = new int[v];

        BFS(adj, s, dest, v, pred, dist);

        // LinkedList to store path
        LinkedList<Integer> path = new LinkedList<Integer>();
        int crawl = dest;
        path.add(crawl);
        while (pred[crawl] != -1) {
            path.add(pred[crawl]);
            crawl = pred[crawl];
        }

        for (int i = path.size() - 2; i >= 0; i--) {
            //moving func
            moving2(target[path.get(i)],quater[path.get(i)]);
        }
        //Check Point
        current_key = path.get(0);
        Curr = target[current_key];
        Shooting_point(current_key);
    }

    // a modified version of BFS that stores predecessor
    // of each vertex in array pred
    // and its distance from source in array dist
    private boolean BFS(ArrayList<ArrayList<Integer>> adj, int src,
                               int dest, int v, int pred[], int dist[])
    {
        // a queue to maintain queue of vertices whose
        // adjacency list is to be scanned as per normal
        // BFS algorithm using LinkedList of Integer type
        LinkedList<Integer> queue = new LinkedList<Integer>();

        // boolean array visited[] which stores the
        // information whether ith vertex is reached
        // at least once in the Breadth first search
        boolean visited[] = new boolean[v];

        // initially all vertices are unvisited
        // so v[i] for all i is false
        // and as no path is yet constructed
        // dist[i] for all i set to infinity
        for (int i = 0; i < v; i++) {
            visited[i] = false;
            dist[i] = Integer.MAX_VALUE;
            pred[i] = -1;
        }

        // now source is first to be visited and
        // distance from source to itself should be 0
        visited[src] = true;
        dist[src] = 0;
        queue.add(src);

        // bfs Algorithm
        while (!queue.isEmpty()) {
            int u = queue.remove();
            for (int i = 0; i < adj.get(u).size(); i++) {
                if (visited[adj.get(u).get(i)] == false) {
                    visited[adj.get(u).get(i)] = true;
                    dist[adj.get(u).get(i)] = dist[u] + 1;
                    pred[adj.get(u).get(i)] = u;
                    queue.add(adj.get(u).get(i));

                    // stopping condition (when we find
                    // our destination)
                    if (adj.get(u).get(i) == dest)
                        return true;
                }
            }
        }
        return false;
    }


    private int find_min2(List<Integer> list, int n){
        double MAX = 1500.001;
        int key = 0;
        for(int i = 0;i<n;i++) {
            int temp = list.get(i);
            double distant = Math.sqrt(Math.pow(Curr[0] - target[temp][0], 2) + Math.pow(Curr[1] - target[temp][1], 2) + Math.pow(Curr[2] - target[temp][2], 2));
            //find min distant Point
            if (distant < MAX) {
                MAX = distant;
                key = temp;
            }
        }
        Log.i("Min Value", "Min target is = " + key);
        return key;
    }

    private boolean moving(double[] pos, Quaternion quaternion) {

        final Point point = new Point(pos[0], pos[1], pos[2]);

        Result result = api.moveTo(point, quaternion, false);

        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 2) {
            result = api.moveTo(point, quaternion, false);
            loopCounter++;
        }
        Log.i("move1[count]", "" + loopCounter);
        return true;
    }

    private boolean moving2(double[] pos, float[] qua) {

        final Point point = new Point(pos[0], pos[1], pos[2]);
        final Quaternion quaternion = new Quaternion(qua[0], qua[1], qua[2], qua[3]);

        Result result = api.moveTo(point, quaternion, false);

        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < 2) {
            result = api.moveTo(point, quaternion, false);
            loopCounter++;
        }
        Log.i("move2[count]", "" + loopCounter);
        return true;
    }
}
