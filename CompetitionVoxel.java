package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.ArrayList;
import java.util.List;

public class CompetitionVoxel {
    VectorF location = new VectorF(0,0,0);
    float danger = 0;
    float path = 0;
    float robot = 0;
    List<CompetitionVoxel> neighbors = new ArrayList<>();

    public boolean isDanger(float threshold){
        return danger > threshold;
    }

    public List<CompetitionVoxel> getSafeNeighbors(double threshold){
        List<CompetitionVoxel> ret = new ArrayList<>();
        for (CompetitionVoxel c: neighbors) {
            if (c != null && c.danger < threshold) {
                ret.add(c);
            }
        }
        return ret;
    }
}
