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

//Java library
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    double[][] target = {{0,0,0},//NULL test pointB
            {11.201, -10.15, 5.472},//tar1
            {10.633, -9.137, 4.462},//tar2
            {10.71, -7.765, 4.42},//{10.7742, -7.7677, 4.42},//tar3
            {10.43, -6.615, 5.203},//tar4
            {11.212, -7.971, 5.53},//tar5
            {11.502, -9.056, 4.94},//tar6
            {11, -9.4, 5.1},//pointA
            {10.912, -8.551, 5.29},//pointB
            {10.894, -7.308, 4.9},//pointC
            {9.815, -9.806, 4.293},//start,
            {11.381944, -8.566172, 3.76203},//QR
            {11.143, -6.760, 4.965},//Goal
            {10.662, - 8.327, 4.854}//pointD
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
            {0f, 0.707f, 0f, 0.707f},//pointC
            {0f, 0f, 0f, 1f},//start
            {0f, 0.707f, 0f, 0.707f},//QR
            {0f, 0f, 0f, 1f},//Goal
            {0f, 0.707f, 0f, 0.707f}//bypass 2 - 3
    };

    double[] Start = {9.815, -9.806, 4.293};

    //x มากไปขวา y มากยิ่งไกล z มากยิ่งต่ำ
    //Group CheckPoint
    double[] P_GroupA = {11,-9.4,5.1};//11.115,-9.2821,4.8101
    double[] P_GroupB = {10.81,-7.37,4.9};//10.61,-7.57,5

    //default Quaternion
    float[] Qua_reset = {0f, 0f, 0f, 1f};
    //Current Point
    double Curr[] = Start;
    double Curr2[] = Start;
    int current_key = 10; // at start

    double[] Goal = {11.143,-6.7607,4.9654};
    Quaternion Qua_Goal = new Quaternion(0f, 0f, -0.707f, 0.707f);
    double[] QR = {11.381944,-8.566172,3.76203};
    Quaternion Qua_QR = new Quaternion(0f, 0.707f, 0f, 0.707f);

    //Compare Value
    int temp = 0;
    int Shooting_count = 0;
    int cnt_checkpoint = 0;

    @Override
    protected void runPlan1(){

        api.startMission();
        solution();

        //For QR Scan

        // turn on the front flash light
        //api.flashlightControlFront(0.05f);

        // get QR code content
        String mQrContent = yourMethod();
        mQrContent = "ASTROBEE";
        // turn off the front flash light
        //api.flashlightControlFront(0.00f);


        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();

        Log.i("Goallll", "Move to ending");
        if(Curr == target[1] || Curr == target[2]){
            moving2(target[13],Qua_reset);
        }
        moving(Goal, Qua_Goal);

        // send mission completion
        api.reportMissionCompletion(mQrContent);
    }


    @Override
    protected void runPlan2(){
        // the mission starts

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
        moving2(target[key],quater[key]);
        Log.i("SHOOTING!!!", "Moving to target : " + key);
        api.laserControl(true);
        Log.i("Snapshot", "target snapshot id : " + key);
        //Shooting_count++;
        api.takeTargetSnapshot(key);
        //api.laserControl(false);
    }

    private void solution() {

        // Node
        int v = 14;

        // Adjacency list for storing which vertices are connected
        ArrayList<ArrayList<Integer>> adj = new ArrayList<ArrayList<Integer>>(v);
        for (int i = 0; i < v; i++) {
            adj.add(new ArrayList<Integer>());
        }

        //| 1 - 6 = target | 7 = A | 8 = B | 9 = C | 10 = start | 12 = end |

        //bypass
        add_edge(adj, 3, 9);
        add_edge(adj, 4, 9);
        add_edge(adj, 6, 7);
        add_edge(adj, 13, 2);
        add_edge(adj, 13, 3);

        //A
        add_edge(adj, 1, 7);
        add_edge(adj, 2, 7);
        add_edge(adj, 5, 7);

        add_edge(adj, 7, 8);
        add_edge(adj, 7, 9);
        add_edge(adj, 7, 10);
        /*
        //B
        add_edge(adj, 1, 8);
        add_edge(adj, 2, 8);
        add_edge(adj, 3, 8);
        add_edge(adj, 4, 8);
        add_edge(adj, 5, 8);
        add_edge(adj, 6, 8);
        add_edge(adj, 8, 10);
        */
        //C
        //add_edge(adj, 1, 9);
        /*
        add_edge(adj, 5, 9);
        add_edge(adj, 6, 9);
        */
        //D
        add_edge(adj, 13, 1);
        add_edge(adj, 13, 4);
        add_edge(adj, 13, 5);
        add_edge(adj, 13, 6);
        add_edge(adj, 13, 7);
        add_edge(adj, 13, 9);
        add_edge(adj, 13, 7);
        add_edge(adj, 13, 10);

        //Point to Point
        add_edge(adj, 1, 4);
        add_edge(adj, 1, 5);
        add_edge(adj, 1, 6);
        add_edge(adj, 2, 5);
        add_edge(adj, 4, 5);
        add_edge(adj, 4, 6);
        add_edge(adj, 5, 6);

        //Start to Point
        add_edge(adj, 1, 10);
        add_edge(adj, 2, 10);
        add_edge(adj, 5, 10);
        add_edge(adj, 6, 10);

        //Goal
        add_edge(adj, 12, 3);
        add_edge(adj, 12, 4);
        add_edge(adj, 12, 5);
        add_edge(adj, 12, 6);
        add_edge(adj, 12, 7);
        add_edge(adj, 12, 9);

        //---------------start--------------\\

        //start == Curr, end == key, v == node

        int loop_counter = 0;

        while (true){

            // get the list of active target id
            List<Integer> list = api.getActiveTargets();
            int n = list.size();

            //debug
            Log.i("klekle", "n = " + n);
            for(int i=0;i<n;i++){
                Log.i("klekle", "kle index " + i + " = " + list.get(i));
            }

            int key,copykey = 0;
            for (int i = 0; i < n; i++) {
                //Cut Function for best case
                /*
                if(Shooting_count >= 6 && cnt_checkpoint > 1){
                    Log.i("Target Break ", "7 targets have been shot");
                    break;
                }*/

                if(Shooting_count >= 7 && cnt_checkpoint > 0){
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
                if(key == 1 && Shooting_count > 6){
                    Log.i("Target Break ", "Last point is target 1");
                    break;
                }
                findShortestDistance(adj, current_key, key, v);
                //checkpoint
                
                if(cnt_checkpoint > 3 && Shooting_count > 4 ){
                    Log.i("Target Break ", "3 Checkpoints already pass");
                    break;
                }
            }


            // get remaining active time and mission time
            List<Long> timeRemaining = api.getTimeRemaining();
            // check the remaining milliseconds of mission time
            if (timeRemaining.get(1) < 60000){
                Log.i("Ending", "ending cause time ");
                break;
            }
            //cut function for worse case
            if (loop_counter == 6 || Shooting_count == 8 || (cnt_checkpoint > 3 && Shooting_count > 4)){
                Log.i("Ending", "Ending function");
                break;
            }
            loop_counter++;
        }
    }

    private void add_edge(ArrayList<ArrayList<Integer>> adj, int i, int j) {
        adj.get(i).add(j);
        adj.get(j).add(i);
    }

    private void findShortestDistance(
            ArrayList<ArrayList<Integer>> adj,
            int s, int dest, int v) {
        // stores predecessor path and distant
        int pred[] = new int[v];
        int dist[] = new int[v];

        BFS(adj, s, dest, v, pred, dist);

        //store path
        LinkedList<Integer> path = new LinkedList<Integer>();
        int tem = dest;
        path.add(tem);
        while (pred[tem] != -1) {
            path.add(pred[tem]);
            tem = pred[tem];
        }

        for (int i = path.size() - 2; i >= 0; i--) {
            int KEY = path.get(i);
            //moving func
            Log.i("Path", "Next Path is " + KEY);
            //Checkpoint break condition

            if(KEY == 7 || KEY == 8 || KEY == 9 || KEY == 13){
                cnt_checkpoint++;
                Log.i("Checkpoint", "Pass : " + cnt_checkpoint + " Checkpoint");
                if(cnt_checkpoint > 3 && Shooting_count > 4){
                    return ;
                }
            }
            moving2(target[KEY],quater[KEY]);
        }
        //Check Point
        current_key = path.get(0);
        Curr = target[current_key];
        Shooting_point(current_key);
        Shooting_count++;
    }

    //BFS stores predecessor
    private boolean BFS(ArrayList<ArrayList<Integer>> adj, int src,
                        int dest, int v, int pred[], int dist[]) {

        LinkedList<Integer> queue = new LinkedList<Integer>();

        boolean visited[] = new boolean[v];

        // set Value default
        for (int i = 0; i < v; i++) {
            visited[i] = false;
            dist[i] = Integer.MAX_VALUE;
            pred[i] = -1;
        }

        //start point
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

                    // found the destination
                    if (adj.get(u).get(i) == dest)
                        return true;
                }
            }
        }
        return false;
    }

    private int find_min2(List<Integer> list, int n){
        double MAX = 1500;
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
