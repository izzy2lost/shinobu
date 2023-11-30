#ifndef MAP_COMPILE_HOOKS_H
#define MAP_COMPILE_HOOKS_H

#include "modules/tbloader/src/tb_loader_singleton.h"
#include "scene/3d/collision_shape_3d.h"

class MapCompileHooks : public TBLoaderHook {
	virtual void post_compile_hook(TBLoader *p_loader, TBLoaderBuildInfo *p_build_info);
};

#endif // MAP_COMPILE_HOOKS_H
