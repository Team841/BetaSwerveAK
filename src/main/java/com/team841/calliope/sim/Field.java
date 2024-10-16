package com.team841.calliope.sim;

import org.dyn4j.dynamics.Body;
import org.dyn4j.world.World;

public class Field {
    protected final World<Body> world;

    public Field(){
        this.world = new World<>();
    }
}
