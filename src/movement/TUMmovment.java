/*
 * Copyright 2010 Aalto University, ComNet
 * Released under GPLv3. See LICENSE.txt for details.
 */
package movement;

import core.Coord;
import core.Settings;
import core.SettingsError;
import core.SimClock;
import movement.map.DijkstraPathFinder;
import movement.map.MapNode;
import movement.map.MapRoute;
import movement.map.PointsOfInterest;

import java.util.List;

/**
 * Map based movement model that uses predetermined paths within the map area.
 * Nodes using this model (can) stop on every route waypoint and find their
 * way to next waypoint using {@link DijkstraPathFinder}. There can be
 * different type of routes; see {@link #ROUTE_TYPE_S}.
 */
public class TUMmovment extends MapBasedMovement implements
	SwitchableMovement {

	/** Per node group setting used for selecting a route file ({@value}) */
	public static final String ROUTE_FILE_S = "routeFile";
	/**
	 * Per node group setting used for selecting a route's type ({@value}).
	 * Integer value from {@link MapRoute} class.
	 */
	public static final String ROUTE_TYPE_S = "routeType";

	/**
	 * Per node group setting for selecting which stop (counting from 0 from
	 * the start of the route) should be the first one. By default, or if a
	 * negative value is given, a random stop is selected.
	 */
	public static final String ROUTE_FIRST_STOP_S = "routeFirstStop";

	/** the Dijkstra shortest path finder */
	private DijkstraPathFinder pathFinder;

	/** Prototype's reference to all routes read for the group */
	private List<MapRoute> allRoutes = null;
	/** next route's index to give by prototype */
	private Integer nextRouteIndex = null;
	/** Index of the first stop for a group of nodes (or -1 for random) */
	private int firstStopIndex = -1;

	/** Route of the movement model's instance */
	private MapRoute route;

	//POI for POI Mode
	private PointsOfInterest pois;

	//Mode carry over
	public int mode = 0;

	//clock of decision making
	int clock = 0;

	// 2 clocktimes, eg. for start and end of lunch
	public int clock_begin;
	public int clock_end;
	
	// node counter
	public int nodecounter = 0;
	public static int total_nodecounter;

	// read those clocks from the settings
	public static final String CLOCK1_S = "clock_begin";
	public static final String CLOCK2_S = "clock_end";

	/**
	 * Creates a new movement model based on a Settings object's settings.
	 * @param settings The Settings object where the settings are read from
	 */
	public TUMmovment(Settings settings) {
		super(settings);
		total_nodecounter = 0;
		String fileName = settings.getSetting(ROUTE_FILE_S);
		String sclock1 = settings.getSetting(CLOCK1_S);
		String sclock2 = settings.getSetting(CLOCK2_S);
		clock_begin = Integer.parseInt(sclock1);
		clock_end = Integer.parseInt(sclock2);
		int type = settings.getInt(ROUTE_TYPE_S);
		allRoutes = MapRoute.readRoutes(fileName, type, getMap());
		nextRouteIndex = 0;
		pathFinder = new DijkstraPathFinder(getOkMapNodeTypes());
		this.route = this.allRoutes.get(this.nextRouteIndex).replicate();
		if (this.nextRouteIndex >= this.allRoutes.size()) {
			this.nextRouteIndex = 0;
		}

		if (settings.contains(ROUTE_FIRST_STOP_S)) {
			this.firstStopIndex = settings.getInt(ROUTE_FIRST_STOP_S);
			if (this.firstStopIndex >= this.route.getNrofStops()) {
				throw new SettingsError("Too high first stop's index (" +
						this.firstStopIndex + ") for route with only " +
						this.route.getNrofStops() + " stops");
			}
		}
		this.pois = new PointsOfInterest(getMap(), getOkMapNodeTypes(),
				settings, rng);
	}

	/**
	 * Copyconstructor. Gives a route to the new movement model from the
	 * list of routes and randomizes the starting position.
	 * @param proto The MapRouteMovement prototype
	 */
	protected TUMmovment(TUMmovment proto) {
		super(proto);
		this.nodecounter = total_nodecounter;
		this.route = proto.allRoutes.get(proto.nextRouteIndex).replicate();
		this.firstStopIndex = proto.firstStopIndex;

		if (firstStopIndex < 0) {
			/* set a random starting position on the route */
			this.route.setNextIndex(rng.nextInt(route.getNrofStops()-1));
		} else {
			/* use the one defined in the config file */
			this.route.setNextIndex(this.firstStopIndex);
		}
		this.clock_begin = proto.clock_begin;
		this.clock_end = proto.clock_end;

		this.pathFinder = proto.pathFinder;
		this.pois = proto.pois;

		proto.nextRouteIndex++; // give routes in order
		if (proto.nextRouteIndex >= proto.allRoutes.size()) {
			proto.nextRouteIndex = 0;
		}
	}

	@Override
	public Path getPath() {
		//get current time for decision making
		clock = SimClock.getIntTime();
		System.out.println(clock + " " + this.nodecounter);
		//2do make decision
		this.mode = 0;

		if (clock > clock_begin && clock < clock_end) {
			// eating
			System.out.println ("eating");
			return getrandomPath();
		}
		else {
			// static workshedule
			return getstaticPath();

		}
	}

	public Path getstaticPath() {
		Path p = new Path(generateSpeed());
		MapNode to = route.nextStop();

		List<MapNode> nodePath = pathFinder.getShortestPath(lastMapNode, to);

		// this assertion should never fire if the map is checked in read phase
		assert nodePath.size() > 0 : "No path from " + lastMapNode + " to " +
			to + ". The simulation map isn't fully connected";

		for (MapNode node : nodePath) { // create a Path from the shortest path
			p.addWaypoint(node.getLocation());
		}

		lastMapNode = to;

		return p;
	}
	public Path getrandomPath() {
		Path p = new Path(generateSpeed());
		MapNode to = pois.selectDestination();

		List<MapNode> nodePath = pathFinder.getShortestPath(lastMapNode, to);

		// this assertion should never fire if the map is checked in read phase
		assert nodePath.size() > 0 : "No path from " + lastMapNode + " to " +
				to + ". The simulation map isn't fully connected";

		for (MapNode node : nodePath) { // create a Path from the shortest path
			p.addWaypoint(node.getLocation());
		}

		lastMapNode = to;

		return p;
	}
	/**
	 * Returns the first stop on the route
	 */
	@Override
	public Coord getInitialLocation() {
		if (lastMapNode == null) {
			lastMapNode = route.nextStop();
		}

		return lastMapNode.getLocation().clone();
	}

	@Override
	public Coord getLastLocation() {
		if (lastMapNode != null) {
			return lastMapNode.getLocation().clone();
		} else {
			return null;
		}
	}


	@Override
	public TUMmovment replicate() {
		total_nodecounter++;
		return new TUMmovment(this);
	}

	/**
	 * Returns the list of stops on the route
	 * @return The list of stops
	 */
	public List<MapNode> getStops() {
		return route.getStops();
	}
}
