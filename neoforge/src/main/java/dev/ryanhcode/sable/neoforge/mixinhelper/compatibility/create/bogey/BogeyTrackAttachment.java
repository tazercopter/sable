package dev.ryanhcode.sable.neoforge.mixinhelper.compatibility.create.bogey;

import com.simibubi.create.content.trains.bogey.BogeySizes;
import dev.ryanhcode.sable.api.physics.handle.RigidBodyHandle;
import dev.ryanhcode.sable.companion.math.JOMLConversion;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.world.phys.Vec3;
import org.joml.Matrix3d;
import org.joml.Quaterniond;
import org.joml.Quaterniondc;
import org.joml.Vector3d;
import org.joml.Vector3dc;

public class BogeyTrackAttachment {

    private final BogeyAxle[] axles;

    public BogeyTrackAttachment(final BlockPos bogeyPos, final Direction.Axis bogeyAxis, final BogeySizes.BogeySize bogeySize) {
        final Vector3dc bogeyCenter = JOMLConversion.toJOML(Vec3.atCenterOf(bogeyPos));
        final Vector3dc bogeyForwardLocal = forwardForAxis(bogeyAxis);
        final Quaterniondc bogeyFrameOrientation = computeBogeyFrameOrientation(bogeyForwardLocal);

        if (bogeySize == BogeySizes.SMALL) {
            final Vector3d back = new Vector3d(bogeyCenter).fma(-1, bogeyForwardLocal);
            final Vector3d front = new Vector3d(bogeyCenter).fma(1, bogeyForwardLocal);
            this.axles = new BogeyAxle[] {
                    new BogeyAxle(back, bogeyForwardLocal, bogeyFrameOrientation),
                    new BogeyAxle(front, bogeyForwardLocal, bogeyFrameOrientation)
            };
        } else {
            this.axles = new BogeyAxle[] {
                    new BogeyAxle(bogeyCenter, bogeyForwardLocal, bogeyFrameOrientation)
            };
        }
    }

    public void physicsTick(final ServerSubLevel subLevel, final RigidBodyHandle handle, final double timeStep) {
        for (final BogeyAxle axle : this.axles) {
            axle.physicsTick(subLevel, handle, timeStep);
        }
    }

    public void onRemoved() {
        for (final BogeyAxle axle : this.axles) {
            axle.onRemoved();
        }
    }

    private static Vector3dc forwardForAxis(final Direction.Axis axis) {
        return switch (axis) {
            case X -> new Vector3d(1.0, 0.0, 0.0);
            case Z -> new Vector3d(0.0, 0.0, 1.0);
            case Y -> new Vector3d(0.0, 1.0, 0.0);
        };
    }

    private static Quaterniondc computeBogeyFrameOrientation(final Vector3dc forward) {
        final Vector3dc up = new Vector3d(0.0, 1.0, 0.0);
        final Quaterniond result = BogeyAxle.buildFrameOrientation(forward, up, new Vector3d(), new Vector3d(), new Vector3d(), new Matrix3d(), new Quaterniond());
        return result != null ? result : new Quaterniond();
    }
}
