package dev.ryanhcode.sable.neoforge.mixinhelper.compatibility.create.bogey;

import com.simibubi.create.Create;
import com.simibubi.create.content.trains.GlobalRailwayManager;
import com.simibubi.create.content.trains.graph.TrackEdge;
import com.simibubi.create.content.trains.graph.TrackGraph;
import com.simibubi.create.content.trains.graph.TrackGraphBounds;
import com.simibubi.create.content.trains.graph.TrackNode;
import com.simibubi.create.content.trains.graph.TrackNodeLocation;
import dev.ryanhcode.sable.Sable;
import dev.ryanhcode.sable.api.physics.constraint.ConstraintJointAxis;
import dev.ryanhcode.sable.api.physics.constraint.generic.GenericConstraintConfiguration;
import dev.ryanhcode.sable.api.physics.constraint.generic.GenericConstraintHandle;
import dev.ryanhcode.sable.api.physics.handle.RigidBodyHandle;
import dev.ryanhcode.sable.companion.math.Pose3dc;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import dev.ryanhcode.sable.sublevel.SubLevel;
import dev.ryanhcode.sable.sublevel.system.SubLevelPhysicsSystem;
import net.minecraft.resources.ResourceKey;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import org.jetbrains.annotations.Nullable;
import org.joml.Matrix3d;
import org.joml.Quaterniond;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

import java.util.EnumSet;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;

public class BogeyAxle {

    private static final double SNAP_RADIUS = 1.2;
    private static final double SNAP_FORWARD_DOT = 0.85;
    private static final double SNAP_UP_DOT = 0.7;
    private static final double FOLLOWER_TANGENT_COMPATIBILITY_DOT = 0.85;
    private static final double LATERAL_DERAIL_FORCE = 1000.0;
    private static final double TIP_DERAIL_DOT_THRESHOLD = 0.5;
    private static final double TRACK_FRICTION_DAMPING = 0.25;
    private static final double ROLL_RESTORING_STIFFNESS = 100.0;
    private static final double ROLL_RESTORING_DAMPING = 20.0;
    private static final double AXLE_REST_OFFSET = 1.45;
    private static final double DEGENERATE_BASIS_EPSILON_SQ = 1.0e-9;
    private static final double EDGE_BOUNDARY_EPSILON = 1.0e-6;
    private static final int BEZIER_SAMPLES = 16;
    private static final int BEZIER_REFINEMENTS = 8;
    private static final double BEZIER_REFINE_WINDOW = 0.25;

    private static final Set<ConstraintJointAxis> LEADER_LOCKED_AXES = EnumSet.of(
            ConstraintJointAxis.LINEAR_Y,
            ConstraintJointAxis.LINEAR_Z,
            ConstraintJointAxis.ANGULAR_Y,
            ConstraintJointAxis.ANGULAR_Z
    );
    private static final Set<ConstraintJointAxis> FOLLOWER_LOCKED_AXES = EnumSet.of(
            ConstraintJointAxis.LINEAR_Y,
            ConstraintJointAxis.LINEAR_Z
    );

    private static final Vector3dc BOGEY_LOCAL_UP = new Vector3d(0.0, 1.0, 0.0);

    private static final Map<ServerSubLevel, BogeyAxle> LEADER_BY_SUBLEVEL = new WeakHashMap<>();

    static @Nullable BogeyAxle currentLeader(final ServerSubLevel subLevel) {
        final BogeyAxle leader = LEADER_BY_SUBLEVEL.get(subLevel);
        return leader != null && leader.isAttached() ? leader : null;
    }

    private final Vector3dc axleAnchorLocal;
    private final Vector3dc bogeyForwardLocal;
    private final Quaterniondc bogeyFrameOrientation;

    private @Nullable GenericConstraintHandle constraint;
    private @Nullable TrackGraph cachedGraph;
    private @Nullable ServerSubLevel cachedHost;
    private @Nullable TrackEdge attachedEdge;
    private double attachedEdgeT;
    private double tangentSign = 1.0;
    private boolean isLeaderRole;
    private @Nullable ServerSubLevel leaderRegistration;

    private final Vector3d scratchAxleWorld = new Vector3d();
    private final Vector3d scratchAxleHostLocal = new Vector3d();
    private final Vector3d scratchBogeyWorldForward = new Vector3d();
    private final Vector3d scratchTangentWorld = new Vector3d();
    private final Vector3d scratchLeaderTangentWorld = new Vector3d();
    private final Vector3d scratchTrackHostLocalPos = new Vector3d();
    private final Vector3d scratchTangent = new Vector3d();
    private final Vector3d scratchNormal = new Vector3d();
    private final Vector3d scratchLateral = new Vector3d();
    private final Vector3d scratchLinearImpulse = new Vector3d();
    private final Vector3d scratchAngularImpulse = new Vector3d();
    private final Quaterniond scratchOrientation = new Quaterniond();
    private final Matrix3d scratchBasis = new Matrix3d();

    public BogeyAxle(final Vector3dc axleAnchorLocal, final Vector3dc bogeyForwardLocal, final Quaterniondc bogeyFrameOrientation) {
        this.axleAnchorLocal = axleAnchorLocal;
        this.bogeyForwardLocal = bogeyForwardLocal;
        this.bogeyFrameOrientation = bogeyFrameOrientation;
    }

    public boolean isAttached() {
        return this.constraint != null && this.constraint.isValid();
    }

    public boolean isLeader() {
        return this.isLeaderRole;
    }

    @Nullable Vector3d currentRailTangentWorld(final Vector3d dest) {
        if (!this.isAttached() || this.attachedEdge == null) {
            return null;
        }
        final Vec3 tangentLevel = this.attachedEdge.getDirectionAt(this.attachedEdgeT * this.attachedEdge.getLength());
        directionToWorld(tangentLevel, this.cachedHost, dest);
        if (this.tangentSign < 0.0) {
            dest.negate();
        }
        return dest;
    }

    public void physicsTick(final ServerSubLevel subLevel, final RigidBodyHandle handle, final double timeStep) {
        final Level level = subLevel.getLevel();
        if (level == null) {
            this.detach();
            return;
        }

        if (this.cachedGraph != null && !graphIsAlive(this.cachedGraph)) {
            this.detach();
            this.cachedGraph = null;
            this.cachedHost = null;
        }

        final BogeyAxle leader = LEADER_BY_SUBLEVEL.get(subLevel);
        final boolean leaderAlive = leader != null && leader.isAttached();
        final boolean shouldBeLeader = !leaderAlive || leader == this;

        if (this.isAttached() && this.isLeaderRole != shouldBeLeader) {
            this.detach();
        }

        final Pose3dc bogeyHostPose = subLevel.logicalPose();
        bogeyHostPose.transformPosition(this.axleAnchorLocal, this.scratchAxleWorld);

        if (this.constraint == null || !this.constraint.isValid()) {
            this.constraint = null;
            this.attemptAttach(subLevel, level, shouldBeLeader);
            return;
        }

        if (this.attachedEdge == null || this.cachedGraph == null) {
            this.detach();
            return;
        }

        bogeyWorldToHostLocal(this.scratchAxleWorld, this.cachedHost, this.scratchAxleHostLocal);

        this.attachedEdgeT = projectOntoEdgeRefine(this.cachedGraph, this.attachedEdge, this.scratchAxleHostLocal, this.attachedEdgeT);

        final boolean pastEnd = this.attachedEdgeT >= 1.0 - EDGE_BOUNDARY_EPSILON
                && isPastEdgeEndpoint(this.attachedEdge, this.attachedEdge.node2, true, this.scratchAxleHostLocal);
        final boolean pastStart = this.attachedEdgeT <= EDGE_BOUNDARY_EPSILON
                && isPastEdgeEndpoint(this.attachedEdge, this.attachedEdge.node1, false, this.scratchAxleHostLocal);

        if (pastStart || pastEnd || this.attachedEdgeT < 0.0 || this.attachedEdgeT > 1.0) {
            final TrackNode pivot = (pastEnd || this.attachedEdgeT > 1.0) ? this.attachedEdge.node2 : this.attachedEdge.node1;
            final EdgeProjection transition = pickConnectedEdge(this.cachedGraph, this.cachedHost, this.attachedEdge, pivot, this.scratchAxleHostLocal);
            if (transition == null) {
                this.detach();
                return;
            }
            this.attachedEdge = transition.edge;
            this.attachedEdgeT = transition.t;
            this.recomputeTangentSign(subLevel);
        }

        this.applyEdgeFrame();
        this.checkForDetachment(subLevel, timeStep);
    }

    public void onRemoved() {
        this.detach();
        this.cachedGraph = null;
        this.cachedHost = null;
    }

    private void attemptAttach(final ServerSubLevel subLevel, final Level level, final boolean asLeader) {
        EdgeProjection nearest = null;
        if (this.cachedGraph != null && graphContainsAxle(this.cachedGraph, this.cachedHost, level, this.scratchAxleWorld)) {
            nearest = findNearestEdgeInGraph(this.cachedGraph, this.cachedHost, level.dimension(), this.scratchAxleWorld);
        }
        if (nearest == null) {
            nearest = findNearestEdgeAcrossAllGraphs(level, this.scratchAxleWorld);
        }
        if (nearest == null) {
            return;
        }

        if (nearest.host == subLevel) {
            return;
        }

        final Vec3 tangentLevel = nearest.edge.getDirectionAt(nearest.t * nearest.edge.getLength());
        directionToWorld(tangentLevel, nearest.host, this.scratchTangentWorld);
        subLevel.logicalPose().transformNormal(this.bogeyForwardLocal, this.scratchBogeyWorldForward).normalize();
        final double signedAlignment = this.scratchBogeyWorldForward.dot(this.scratchTangentWorld);
        if (Math.abs(signedAlignment) < SNAP_FORWARD_DOT) {
            return;
        }
        this.tangentSign = signedAlignment >= 0.0 ? 1.0 : -1.0;

        final Vec3 normalLevel = nearest.edge.getNormal(nearest.graph, nearest.t);
        directionToWorld(normalLevel, nearest.host, this.scratchTangentWorld);
        subLevel.logicalPose().transformNormal(BOGEY_LOCAL_UP, this.scratchBogeyWorldForward).normalize();
        final double upAlignment = this.scratchBogeyWorldForward.dot(this.scratchTangentWorld);
        if (upAlignment < SNAP_UP_DOT) {
            return;
        }

        if (!asLeader) {
            final BogeyAxle leader = currentLeader(subLevel);
            if (leader == null) {
                return;
            }
            final Vector3d leaderTangent = leader.currentRailTangentWorld(this.scratchLeaderTangentWorld);
            if (leaderTangent == null) {
                return;
            }
            final Vec3 candidateTangentLevel = nearest.edge.getDirectionAt(nearest.t * nearest.edge.getLength());
            directionToWorld(candidateTangentLevel, nearest.host, this.scratchTangentWorld);
            if (this.tangentSign < 0.0) {
                this.scratchTangentWorld.negate();
            }
            if (this.scratchTangentWorld.dot(leaderTangent) < FOLLOWER_TANGENT_COMPATIBILITY_DOT) {
                return;
            }
        }

        final Quaterniond frameOrientation = this.computeRailOrientation(nearest.graph, nearest.edge, nearest.t);
        if (frameOrientation == null) {
            return;
        }

        this.cachedGraph = nearest.graph;
        this.cachedHost = nearest.host;
        this.attachedEdge = nearest.edge;
        this.attachedEdgeT = nearest.t;

        this.computeAxleRestPosition(nearest.edge, nearest.graph, nearest.t, this.scratchTrackHostLocalPos);

        final GenericConstraintConfiguration config = new GenericConstraintConfiguration(
                this.axleAnchorLocal,
                this.scratchTrackHostLocalPos,
                this.bogeyFrameOrientation,
                frameOrientation,
                asLeader ? LEADER_LOCKED_AXES : FOLLOWER_LOCKED_AXES
        );

        this.constraint = SubLevelPhysicsSystem.require(level)
                .getPipeline()
                .addConstraint(subLevel, nearest.host, config);
        if (this.constraint == null) {
            return;
        }

        this.constraint.setContactsEnabled(false);
        this.constraint.setMotor(ConstraintJointAxis.LINEAR_X, 0.0, 0.0, TRACK_FRICTION_DAMPING, false, 0.0);
        if (asLeader) {
            this.constraint.setMotor(ConstraintJointAxis.ANGULAR_X, 0.0, ROLL_RESTORING_STIFFNESS, ROLL_RESTORING_DAMPING, false, 0.0);
        }

        this.isLeaderRole = asLeader;
        if (asLeader) {
            this.leaderRegistration = subLevel;
            LEADER_BY_SUBLEVEL.put(subLevel, this);
        } else {
            this.leaderRegistration = null;
        }
    }

    private void applyEdgeFrame() {
        final Quaterniond frameOrientation = this.computeRailOrientation(this.cachedGraph, this.attachedEdge, this.attachedEdgeT);
        if (frameOrientation == null) {
            return;
        }
        this.computeAxleRestPosition(this.attachedEdge, this.cachedGraph, this.attachedEdgeT, this.scratchTrackHostLocalPos);
        this.constraint.setFrame2(this.scratchTrackHostLocalPos, frameOrientation);
    }

    private void computeAxleRestPosition(final TrackEdge edge, final TrackGraph graph, final double t, final Vector3d dest) {
        final Vec3 trackLevel = edge.getPosition(graph, t);
        final Vec3 railNormalRaw = edge.getNormal(graph, t);
        final double normalLenSq = railNormalRaw.lengthSqr();
        final double nx, ny, nz;
        if (!(normalLenSq > DEGENERATE_BASIS_EPSILON_SQ)) {
            nx = 0.0; ny = 1.0; nz = 0.0;
        } else {
            final double invLen = 1.0 / Math.sqrt(normalLenSq);
            nx = railNormalRaw.x * invLen;
            ny = railNormalRaw.y * invLen;
            nz = railNormalRaw.z * invLen;
        }
        dest.set(
                trackLevel.x + nx * AXLE_REST_OFFSET,
                trackLevel.y + ny * AXLE_REST_OFFSET,
                trackLevel.z + nz * AXLE_REST_OFFSET
        );
    }

    private void checkForDetachment(final ServerSubLevel subLevel, final double timeStep) {
        this.constraint.getJointImpulses(this.scratchLinearImpulse, this.scratchAngularImpulse);
        final double lateralImpulse = Math.abs(this.scratchLinearImpulse.y);
        final double verticalImpulse = this.scratchLinearImpulse.z;
        if (verticalImpulse < 0.0 || lateralImpulse > LATERAL_DERAIL_FORCE * timeStep) {
            this.detach();
            return;
        }
        if (!this.isLeaderRole) {
            return;
        }
        subLevel.logicalPose().transformNormal(BOGEY_LOCAL_UP, this.scratchBogeyWorldForward).normalize();
        final Vec3 railNormalLevel = this.attachedEdge.getNormal(this.cachedGraph, this.attachedEdgeT);
        directionToWorld(railNormalLevel, this.cachedHost, this.scratchTangentWorld);
        final double tipAlignment = this.scratchBogeyWorldForward.dot(this.scratchTangentWorld);
        if (tipAlignment < TIP_DERAIL_DOT_THRESHOLD) {
            this.detach();
        }
    }

    private void detach() {
        if (this.constraint != null && this.constraint.isValid()) {
            this.constraint.remove();
        }
        this.constraint = null;
        this.attachedEdge = null;
        this.attachedEdgeT = 0.0;
        if (this.isLeaderRole && this.leaderRegistration != null) {
            LEADER_BY_SUBLEVEL.remove(this.leaderRegistration, this);
        }
        this.isLeaderRole = false;
        this.leaderRegistration = null;
    }

    private void recomputeTangentSign(final ServerSubLevel subLevel) {
        final Vec3 tangentLevel = this.attachedEdge.getDirectionAt(this.attachedEdgeT * this.attachedEdge.getLength());
        directionToWorld(tangentLevel, this.cachedHost, this.scratchTangentWorld);
        subLevel.logicalPose().transformNormal(this.bogeyForwardLocal, this.scratchBogeyWorldForward).normalize();
        this.tangentSign = this.scratchBogeyWorldForward.dot(this.scratchTangentWorld) >= 0.0 ? 1.0 : -1.0;
    }

    private @Nullable Quaterniond computeRailOrientation(final TrackGraph graph, final TrackEdge edge, final double t) {
        final Vec3 tangentVec = edge.getDirectionAt(t * edge.getLength());
        final Vec3 normalVec = edge.getNormal(graph, t);
        this.scratchTangent.set(tangentVec.x * this.tangentSign, tangentVec.y * this.tangentSign, tangentVec.z * this.tangentSign);
        this.scratchNormal.set(normalVec.x, normalVec.y, normalVec.z);
        return buildFrameOrientation(this.scratchTangent, this.scratchNormal, this.scratchTangent, this.scratchNormal, this.scratchLateral, this.scratchBasis, this.scratchOrientation);
    }

    static @Nullable Quaterniond buildFrameOrientation(final Vector3dc forward, final Vector3dc up, final Vector3d scratchForward, final Vector3d scratchUp, final Vector3d scratchLateral, final Matrix3d scratchBasis, final Quaterniond dest) {
        scratchForward.set(forward);
        scratchUp.set(up);
        if (!isFinite(scratchForward) || !isFinite(scratchUp)) {
            return null;
        }
        scratchForward.normalize();
        scratchUp.normalize();
        if (!isFinite(scratchForward) || !isFinite(scratchUp)) {
            return null;
        }
        scratchUp.cross(scratchForward, scratchLateral);
        final double lateralLenSq = scratchLateral.lengthSquared();
        if (!(lateralLenSq > DEGENERATE_BASIS_EPSILON_SQ)) {
            return null;
        }
        scratchLateral.normalize();
        scratchForward.cross(scratchLateral, scratchUp).normalize();
        scratchBasis.set(
                scratchForward.x, scratchForward.y, scratchForward.z,
                scratchLateral.x, scratchLateral.y, scratchLateral.z,
                scratchUp.x,      scratchUp.y,      scratchUp.z
        );
        return dest.setFromNormalized(scratchBasis);
    }

    private static boolean isFinite(final Vector3dc v) {
        return Double.isFinite(v.x()) && Double.isFinite(v.y()) && Double.isFinite(v.z());
    }

    private static void bogeyWorldToHostLocal(final Vector3dc bogeyWorld, final @Nullable ServerSubLevel host, final Vector3d dest) {
        if (host == null) {
            dest.set(bogeyWorld);
        } else {
            host.logicalPose().transformPositionInverse(bogeyWorld, dest);
        }
    }

    private static void directionToWorld(final Vec3 levelDir, final @Nullable ServerSubLevel host, final Vector3d dest) {
        dest.set(levelDir.x, levelDir.y, levelDir.z);
        if (host != null) {
            host.logicalPose().transformNormal(dest);
        }
        dest.normalize();
    }

    private static @Nullable ServerSubLevel resolveHost(final TrackGraph graph, final Level level) {
        final ResourceKey<Level> dim = level.dimension();
        for (final TrackNodeLocation loc : graph.getNodes()) {
            if (!loc.dimension.equals(dim)) {
                continue;
            }
            final SubLevel containing = Sable.HELPER.getContaining(level, loc.getLocation());
            return containing instanceof ServerSubLevel server ? server : null;
        }
        return null;
    }

    private static boolean graphIsAlive(final TrackGraph graph) {
        final GlobalRailwayManager railways = Create.RAILWAYS;
        return railways != null && railways.trackNetworks != null && railways.trackNetworks.get(graph.id) == graph;
    }

    private static boolean graphContainsAxle(final TrackGraph graph, final @Nullable ServerSubLevel host, final Level level, final Vector3dc axleWorld) {
        final TrackGraphBounds bounds = graph.getBounds(level);
        if (bounds == null || bounds.box == null) {
            return false;
        }
        final AABB inflated = bounds.box.inflate(SNAP_RADIUS);
        if (host == null) {
            return inflated.contains(axleWorld.x(), axleWorld.y(), axleWorld.z());
        }
        final Vector3d hostLocal = new Vector3d();
        host.logicalPose().transformPositionInverse(axleWorld, hostLocal);
        return inflated.contains(hostLocal.x, hostLocal.y, hostLocal.z);
    }

    private record EdgeProjection(TrackGraph graph, @Nullable ServerSubLevel host, TrackEdge edge, double t, double distanceSqWorld) {}

    private static @Nullable EdgeProjection findNearestEdgeAcrossAllGraphs(final Level level, final Vector3dc axleWorld) {
        final GlobalRailwayManager railways = Create.RAILWAYS;
        if (railways == null || railways.trackNetworks == null) {
            return null;
        }
        EdgeProjection best = null;
        for (final TrackGraph graph : railways.trackNetworks.values()) {
            final ServerSubLevel host = resolveHost(graph, level);
            if (!graphContainsAxle(graph, host, level, axleWorld)) {
                continue;
            }
            final EdgeProjection candidate = findNearestEdgeInGraph(graph, host, level.dimension(), axleWorld);
            if (candidate != null && (best == null || candidate.distanceSqWorld < best.distanceSqWorld)) {
                best = candidate;
            }
        }
        return best;
    }

    private static @Nullable EdgeProjection findNearestEdgeInGraph(final TrackGraph graph, final @Nullable ServerSubLevel host, final ResourceKey<Level> dimension, final Vector3dc axleWorld) {
        final Vector3d axleHostLocal = new Vector3d();
        bogeyWorldToHostLocal(axleWorld, host, axleHostLocal);

        EdgeProjection best = null;
        for (final TrackNodeLocation location : graph.getNodes()) {
            if (!location.dimension.equals(dimension)) {
                continue;
            }
            final TrackNode node = graph.locateNode(location);
            if (node == null) {
                continue;
            }
            final Map<TrackNode, TrackEdge> connections = graph.getConnectionsFrom(node);
            if (connections == null) {
                continue;
            }
            for (final TrackEdge edge : connections.values()) {
                if (edge.isInterDimensional()) {
                    continue;
                }
                final double t = projectOntoEdgeBlind(graph, edge, axleHostLocal);
                final double clampedT = Math.max(0.0, Math.min(1.0, t));
                final Vec3 candidatePoint = edge.getPosition(graph, clampedT);
                final double distSq = squareDistance(axleHostLocal, candidatePoint);
                if (distSq <= SNAP_RADIUS * SNAP_RADIUS && (best == null || distSq < best.distanceSqWorld)) {
                    best = new EdgeProjection(graph, host, edge, clampedT, distSq);
                }
            }
        }
        return best;
    }

    private static @Nullable EdgeProjection pickConnectedEdge(final TrackGraph graph, final @Nullable ServerSubLevel host, final TrackEdge fromEdge, final TrackNode pivot, final Vector3dc axleHostLocal) {
        final Map<TrackNode, TrackEdge> connections = graph.getConnectionsFrom(pivot);
        if (connections == null) {
            return null;
        }
        EdgeProjection best = null;
        for (final TrackEdge candidate : connections.values()) {
            if (candidate.isInterDimensional() || isSamePhysicalEdge(candidate, fromEdge)) {
                continue;
            }
            final double t = projectOntoEdgeBlind(graph, candidate, axleHostLocal);
            final double clampedT = Math.max(0.0, Math.min(1.0, t));
            final Vec3 closest = candidate.getPosition(graph, clampedT);
            final double distSq = squareDistance(axleHostLocal, closest);
            if (best == null || distSq < best.distanceSqWorld) {
                best = new EdgeProjection(graph, host, candidate, clampedT, distSq);
            }
        }
        return best;
    }

    private static boolean isSamePhysicalEdge(final TrackEdge a, final TrackEdge b) {
        return (a.node1 == b.node1 && a.node2 == b.node2) || (a.node1 == b.node2 && a.node2 == b.node1);
    }

    private static boolean isPastEdgeEndpoint(final TrackEdge edge, final TrackNode node, final boolean atEnd, final Vector3dc axleHostLocal) {
        final Vec3 nodePos = node.getLocation().getLocation();
        final Vec3 forwardTangent = edge.getDirectionAt(atEnd ? edge.getLength() : 0.0);
        final double dx = axleHostLocal.x() - nodePos.x;
        final double dy = axleHostLocal.y() - nodePos.y;
        final double dz = axleHostLocal.z() - nodePos.z;
        final double dot = dx * forwardTangent.x + dy * forwardTangent.y + dz * forwardTangent.z;
        return atEnd ? dot > 0.0 : dot < 0.0;
    }

    private static double projectOntoEdgeBlind(final TrackGraph graph, final TrackEdge edge, final Vector3dc hostLocalPos) {
        if (!edge.isTurn()) {
            return projectOntoSegment(edge.getPosition(graph, 0.0), edge.getPosition(graph, 1.0), hostLocalPos);
        }
        double bestT = 0.5;
        double bestDistSq = Double.MAX_VALUE;
        for (int i = 0; i <= BEZIER_SAMPLES; i++) {
            final double t = (double) i / BEZIER_SAMPLES;
            final double distSq = squareDistance(hostLocalPos, edge.getPosition(graph, t));
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                bestT = t;
            }
        }
        return refineBezierProjection(graph, edge, hostLocalPos, bestT, bestDistSq, 1.0 / BEZIER_SAMPLES);
    }

    private static double projectOntoEdgeRefine(final TrackGraph graph, final TrackEdge edge, final Vector3dc hostLocalPos, final double initialT) {
        if (!edge.isTurn()) {
            return projectOntoSegment(edge.getPosition(graph, 0.0), edge.getPosition(graph, 1.0), hostLocalPos);
        }
        final double bestT = Math.max(0.0, Math.min(1.0, initialT));
        final double bestDistSq = squareDistance(hostLocalPos, edge.getPosition(graph, bestT));
        return refineBezierProjection(graph, edge, hostLocalPos, bestT, bestDistSq, BEZIER_REFINE_WINDOW);
    }

    private static double refineBezierProjection(final TrackGraph graph, final TrackEdge edge, final Vector3dc hostLocalPos, double bestT, double bestDistSq, double window) {
        for (int refinement = 0; refinement < BEZIER_REFINEMENTS; refinement++) {
            final double leftT = Math.max(0.0, bestT - window);
            final double rightT = Math.min(1.0, bestT + window);
            final double leftDistSq = squareDistance(hostLocalPos, edge.getPosition(graph, leftT));
            final double rightDistSq = squareDistance(hostLocalPos, edge.getPosition(graph, rightT));
            if (leftDistSq < bestDistSq) {
                bestDistSq = leftDistSq;
                bestT = leftT;
            }
            if (rightDistSq < bestDistSq) {
                bestDistSq = rightDistSq;
                bestT = rightT;
            }
            window *= 0.5;
        }
        return bestT;
    }

    private static double projectOntoSegment(final Vec3 a, final Vec3 b, final Vector3dc point) {
        final double dx = b.x - a.x;
        final double dy = b.y - a.y;
        final double dz = b.z - a.z;
        final double lengthSq = dx * dx + dy * dy + dz * dz;
        if (lengthSq < 1.0e-12) {
            return 0.0;
        }
        final double px = point.x() - a.x;
        final double py = point.y() - a.y;
        final double pz = point.z() - a.z;
        return (px * dx + py * dy + pz * dz) / lengthSq;
    }

    private static double squareDistance(final Vector3dc point, final Vec3 other) {
        final double dx = point.x() - other.x;
        final double dy = point.y() - other.y;
        final double dz = point.z() - other.z;
        return dx * dx + dy * dy + dz * dz;
    }
}
