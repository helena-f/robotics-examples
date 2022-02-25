package org.firstinspires.ftc.teamcode;


import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.media.Image;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import android.graphics.Bitmap;
import android.os.Environment;
import android.provider.MediaStore;

import com.qualcomm.robotcore.util.Range;

public class CompetitionField extends AStar<CompetitionVoxel>{

    protected CompetitionVoxel goal;
    CompetitionVoxel [][]voxels = null;
    VectorF origin;
    float voxelSize = 1;
    public float minX = 0;
    public float minY = 0;
    public float maxX = 0;
    public float maxY = 0;

    public CompetitionField(){
        origin = new VectorF(0,0,0);
        create(96,142,1.0f); //full field: 142x142
    }

    public CompetitionField(int w, int h, float vSize){
        origin = new VectorF(0,0,0);
        create(w,h,vSize);
    }

    protected void create(int width, int height, float vSize){
        voxelSize = vSize;
        minX = 0;
        minY = 0;
        // bitmap (0,0) is at top left corner.
        // Voxel is right-hand system. the (0,0) is at bottom, right, up is x, left is y
        float yMax = height*voxelSize;
        float xMax = width*voxelSize;
        maxX = xMax;
        maxY = yMax;
        voxels = new CompetitionVoxel[height][width];
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                CompetitionVoxel c = new CompetitionVoxel();
                c.location = origin.added(new VectorF(yMax-y*voxelSize, xMax-x*voxelSize, 0));
                voxels[y][x] = c;
            }
        }

        // second pass build logic for AStar based on geographic
        for (int y = 0; y < voxels.length; y++) {
            for (int x = 0; x < voxels[y].length; x++) {
                CompetitionVoxel v = voxels[y][x];
                v.neighbors.clear();
                // left
                if(x-1>=0){
                    v.neighbors.add(voxels[y][x-1]);
                }
                // right
                if(x+1<voxels[y].length){
                    v.neighbors.add(voxels[y][x+1]);
                }
                // up
                if(y-1>=0){
                    v.neighbors.add(voxels[y-1][x]);
                }
                // down
                if(y+1<voxels.length){
                    v.neighbors.add(voxels[y+1][x]);
                }
                if(x-1>=0 && y-1>=0) {
                    v.neighbors.add(voxels[y-1][x-1]);
                }
                if(x+1<voxels[y].length && y-1>=0) {
                    v.neighbors.add(voxels[y-1][x+1]);
                }
                if(x-1>=0 && y+1<voxels.length) {
                    v.neighbors.add(voxels[y+1][x-1]);
                }
                if(x+1<voxels[y].length && y+1<voxels.length) {
                    v.neighbors.add(voxels[y+1][x+1]);
                }
            }
        }

    }

    /**
     *
     * @param bitmapImage
     */
    public void loadFromImage(Bitmap bitmapImage, float vSize){
//        ByteBuffer buffer = image.getPlanes()[0].getBuffer();
//        byte[] bytes = new byte[buffer.capacity()];
//        buffer.get(bytes);
//        Bitmap bitmapImage = BitmapFactory.decodeByteArray(bytes, 0, bytes.length, null);

        int height = bitmapImage.getHeight();
        int width = bitmapImage.getWidth();
        create(width, height, vSize);

        // bitmap (0,0) is at top left corner.
        // Voxel is right-hand system. the (0,0) is at bottom, right
        //
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                CompetitionVoxel c = voxels[x][y];
                c.danger = Color.red(bitmapImage.getPixel(x, y));
                c.path = Color.green(bitmapImage.getPixel(x, y));
                c.robot = Color.blue(bitmapImage.getPixel(x, y));
            }
        }
    }

    public Bitmap toImage(){
        if (voxels!=null) {
            Bitmap image = Bitmap.createBitmap(voxels[0].length, voxels.length, Bitmap.Config.ARGB_8888);
            for (int y = 0; y < voxels.length; y++) {
                for (int x = 0; x < voxels[y].length; x++) {
                    CompetitionVoxel v = voxels[y][x];
                    image.setPixel(x, y, Color.argb(255,
                            (int) Range.clip(v.danger * 255, 0, 255),
                            (int) Range.clip(v.path * 255, 0, 255),
                            (int) Range.clip(v.robot * 255, 0, 255)));
                }
            }
            return image;
        } else {
            return Bitmap.createBitmap(voxels[0].length, voxels.length, Bitmap.Config.ARGB_8888);
        }
    }

    public void shiftOrigin(VectorF newOrigin) {
        origin.subtract(newOrigin);
        for (int y = 0; y < voxels.length; y++) {
            for (int x = 0; x < voxels[y].length; x++) {
                voxels[y][x].location.subtract(newOrigin);
            }
        }
        minX -= newOrigin.get(0);
        maxX -= newOrigin.get(0);
        minY -= newOrigin.get(1);
        maxY -= newOrigin.get(1);
    }

    public void markRobotLocation (VectorF location){
        CompetitionVoxel v = getVoxel(location);
        if (v!=null) {
            v.robot = 1.0f;
        }
    }

    public void clearRobotLocation (VectorF location){
        CompetitionVoxel v = getVoxel(location);
        if (v!=null) {
            v.robot = 1.0f;
        }
    }

    public void clearRobot(){
        int width = voxels[0].length;
        int height = voxels.length;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                CompetitionVoxel c = voxels[y][x];
                c.robot = 0;
            }
        }
    }

    public void markDangerLocation (VectorF location){
        CompetitionVoxel v = getVoxel(location);
        if (v!=null) {
            v.danger = 1.0f;
        }
    }

    public void clearDangerLocation (VectorF location){
        CompetitionVoxel v = getVoxel(location);
        if (v!=null) {
            v.danger = 0f;
        }
    }

    public void clearDanger(){
        int width = voxels[0].length;
        int height = voxels.length;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                CompetitionVoxel c = voxels[y][x];
                c.danger = 0;
            }
        }
    }

    public void fillDanger(VectorF danger, float dangerRadius, float dangerValue){
        int width = voxels[0].length;
        int height = voxels.length;
        float dangerX = danger.get(0);
        float dangerY = danger.get(1);
        float r2 = dangerRadius*dangerRadius;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // if within danger range
                if ( (x-dangerX)*(x-dangerX)+(y-dangerY)*(y-dangerY) - r2 < 0) {
                    voxels[y][x].danger = dangerValue;
                } else {
                    voxels[y][x].danger = 0;
                }
            }
        }
    }

    public void addDanger(VectorF danger, float dangerRadius, float dangerValue){
        int width = voxels[0].length;
        int height = voxels.length;
        float dangerX = danger.get(0);
        float dangerY = danger.get(1);
        float r2 = dangerRadius*dangerRadius;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // if within danger range
                if ( (x-dangerX)*(x-dangerX)+(y-dangerY)*(y-dangerY) - r2 < 0) {
                    voxels[y][x].danger += dangerValue;
                }
            }
        }
    }

    public void markPathLocation (VectorF location){
        CompetitionVoxel v = getVoxel(location);
        if (v!=null) {
            v.path = 1.0f;
        }
    }

    public void clearPathLocation (VectorF location){
        CompetitionVoxel v = getVoxel(location);
        if (v!=null) {
            v.path = 0f;
        }
    }

    public void setPath(List<VectorF> listF){
        clearPath();
        for (VectorF l : listF){
            markPathLocation(l);
        }
    }

    public void clearPath(){
        int width = voxels[0].length;
        int height = voxels.length;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                CompetitionVoxel c = voxels[y][x];
                c.path = 0;
            }
        }
    }

    /**
     *
     * @param location
     * @return voxel
     */
    public CompetitionVoxel getVoxel(VectorF location){
        if (voxels!=null) {
            int width = voxels[0].length;
            int height = voxels.length;

            VectorF shifted = location.subtracted(origin);
            int x = width-Math.round(shifted.get(1)/voxelSize);
            int y = height- Math.round(shifted.get(0)/voxelSize);

            if (x>=0 && x<width && y>=0 && y<height){
                return voxels[y][x];
            }
        }
        return new CompetitionVoxel();
    }

    /**
     *
     * @param location
     * @return the closest voxel, an expensive way to get voxel location
     */
    CompetitionVoxel getClosetVoxel(VectorF location){
        double closest = Double.MAX_VALUE;
        double current = 0;
        CompetitionVoxel c = null;
        for (int y = 0; y < voxels.length; y++) {
            for (int x = 0; x < voxels[y].length; x++) {
                CompetitionVoxel v = voxels[y][x];
                current = v.location.subtracted(location).magnitude();
                if (current < closest) {
                    c = v;
                    closest = current;
                }
            }
        }
        return c;
    }

    public List<VectorF> smoothPath(List<VectorF> path){
        List<VectorF> smoothP = new ArrayList<>();
        VectorF a = null;
        VectorF b = null;
        VectorF d = null;
        VectorF e = null;
        int count =0;
        for (VectorF c : path) {
            if (count <1){
                smoothP.add(c);
                a=c;
            } else if (count <2 ){
                smoothP.add(c);
                b = c;
            } else if (count <3 ){
                d = c;
            } else if (count <4 ) {
                e = c;
            } else if (count < path.size()-1){
                smoothP.add(c.added(a).added(b).added(d).added(e).multiplied(0.2f));
                a = b;
                b = d;
                d = e;
                e = c;
            } else {
                smoothP.add(c.added(a).added(b).added(d).added(e).multiplied(0.2f));
                smoothP.add(d);
                smoothP.add(e);
            }
            count++;
        }
        return smoothP;
    }

    public List<VectorF> pathToWaypoints(List<CompetitionVoxel> competitionVoxelList) {
        List<VectorF> listF = new ArrayList<>();
        if (competitionVoxelList!=null) {
            for (CompetitionVoxel c : competitionVoxelList) {
                listF.add(c.location);
            }
        }
        return listF;
    }

    public void saveToPhoto(Bitmap image) {
        MediaStore.Images.Media.insertImage(AppUtil.getDefContext().getContentResolver(),
                image, "Competition Field Map",
                "Competition field map with obstacles, robot, and path");
    }

    public List<VectorF> computeWaypoints(VectorF from, VectorF to){
        CompetitionVoxel f = getVoxel(from);
        CompetitionVoxel t = getVoxel(to);
        if (from != null && to != null){
            return pathToWaypoints(computePath(f, t));
        }
        return new ArrayList<>();
    }

    public List<CompetitionVoxel> computePath(CompetitionVoxel from, CompetitionVoxel to){
        goal = to;
        return compute(from);
    }

    /**
     * Check if the current node is a goal for the problem.
     *
     * @param node The node to check.
     * @return <code>true</code> if it is a goal, <code>false</else> otherwise.
     */
    protected boolean isGoal(CompetitionVoxel node) {
        return node == goal;
    }

    /**
     * Cost for the operation to go to <code>to</code> from
     * <code>from</from>.
     *
     * @param from The node we are leaving.
     * @param to The node we are reaching.
     * @return The cost of the operation.
     */
    protected Double g(CompetitionVoxel from, CompetitionVoxel to){
        return (double) to.danger + from.location.subtracted(to.location).magnitude();
    }

    /**
     * Estimated cost to reach a goal node.
     * An admissible heuristic never gives a cost bigger than the real
     * one.
     * <code>from</from>.
     *
     * @param from The node we are leaving.
     * @param to The node we are reaching.
     * @return The estimated cost to reach an object.
     */
    protected Double h(CompetitionVoxel from, CompetitionVoxel to) {
        return (double) from.location.subtracted(to.location).magnitude();
    }

    /**
     * Generate the successors for a given node.
     *
     * @param node The node we want to expand.
     * @return A list of possible next steps.
     */
    protected List<CompetitionVoxel> generateSuccessors(CompetitionVoxel node){
        return node.getSafeNeighbors(0.5);
    }

}
