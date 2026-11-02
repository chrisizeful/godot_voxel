#ifndef ZN_GODOT_NAVIGATION_SERVER_3D_H
#define ZN_GODOT_NAVIGATION_SERVER_3D_H

#if defined(ZN_GODOT)
#include "../core/version.h"
#include <servers/navigation_server_3d.h>
#elif defined(ZN_GODOT_EXTENSION)
#include <godot_cpp/classes/navigation_server3d.hpp>
using namespace godot;
#endif

namespace zylann::godot {

inline void free_navigation_server_rid(NavigationServer3D &ns, const RID &rid) {
#if defined(ZN_GODOT)
#if GODOT_VERSION_MAJOR == 4 && GODOT_VERSION_MINOR <= 5
	ns.free(rid);
#else
	ns.free_rid(rid);
#endif

#elif defined(ZN_GODOT_EXTENSION)
	ns.free_rid(rid);
#endif
}

} // namespace zylann::godot

#endif // ZN_GODOT_NAVIGATION_SERVER_3D_H
