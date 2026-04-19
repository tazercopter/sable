package dev.ryanhcode.sable.neoforge.mixin.compatibility.create.bogey;

import com.simibubi.create.content.trains.bogey.AbstractBogeyBlock;
import com.simibubi.create.content.trains.bogey.AbstractBogeyBlockEntity;
import com.simibubi.create.content.trains.bogey.BogeySizes;
import dev.ryanhcode.sable.api.block.BlockEntitySubLevelActor;
import dev.ryanhcode.sable.api.physics.handle.RigidBodyHandle;
import dev.ryanhcode.sable.neoforge.mixinhelper.compatibility.create.bogey.BogeyTrackAttachment;
import dev.ryanhcode.sable.sublevel.ServerSubLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.level.block.entity.BlockEntityType;
import net.minecraft.world.level.block.state.BlockState;
import org.jetbrains.annotations.Nullable;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;

@Mixin(AbstractBogeyBlockEntity.class)
public abstract class AbstractBogeyBlockEntityMixin extends BlockEntity implements BlockEntitySubLevelActor {

    @Unique
    private @Nullable BogeyTrackAttachment sable$attachment;

    public AbstractBogeyBlockEntityMixin(final BlockEntityType<?> type, final BlockPos pos, final BlockState state) {
        super(type, pos, state);
    }

    @Override
    public void sable$physicsTick(final ServerSubLevel subLevel, final RigidBodyHandle handle, final double timeStep) {
        if (this.sable$attachment == null) {
            final BlockState state = this.getBlockState();
            final Direction.Axis axis = state.getValue(AbstractBogeyBlock.AXIS);
            final BogeySizes.BogeySize size = ((AbstractBogeyBlock<?>) state.getBlock()).getSize();
            this.sable$attachment = new BogeyTrackAttachment(this.getBlockPos(), axis, size);
        }
        this.sable$attachment.physicsTick(subLevel, handle, timeStep);
    }

    @Override
    public void setRemoved() {
        if (this.sable$attachment != null) {
            this.sable$attachment.onRemoved();
            this.sable$attachment = null;
        }
        super.setRemoved();
    }
}
