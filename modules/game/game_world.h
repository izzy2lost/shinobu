#ifndef GAME_MAP_H
#define GAME_MAP_H

#include "scene/3d/node_3d.h"

class GameWorldState : public RefCounted {
public:
    enum AlertStatus {
        ALERT_CLEAR, // Everything is clear
        ALERT_LOST, // Player has been spotted but isn't in view
        ALERT_SEEN // Player's location is known
    };
private:
    AlertStatus alert_status;
};

// Central game coordinator
class HBGameWorld : public Node3D {
    GDCLASS(HBGameWorld, Node3D);

    Transform3D player_start_transform;

public:


    void set_player_start_transform(const Transform3D &p_transform);
    void spawn_player();
};

#endif // GAME_MAP_H
