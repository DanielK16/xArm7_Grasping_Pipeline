#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

namespace mtc = moveit::task_constructor;

class SimpleMTCNode : public rclcpp::Node {
public:
    SimpleMTCNode() : Node("simple_mtc_node") {
        // MTC Task initialisieren
        task_.stages()->setName("Single Move Task");
        task_.loadRobotModel(shared_from_this());

        // Solver: Wir nutzen OMPL (PipelinePlanner) um Kollisionen zu umfahren
        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
        sampling_planner->setPlannerId("RRTConnectkConfigDefault"); 
        // oder "PTP" falls du Pilz nutzt

        // ---------------------------------------------------
        // 1. HINDERNIS ERSTELLEN (Planning Scene)
        // ---------------------------------------------------
        createObstacle();
        
        // ---------------------------------------------------
        // 2. STAGE 1: Startzustand
        // ---------------------------------------------------
        auto stage_current = std::make_unique<mtc::stages::CurrentState>("Start");
        task_.add(std::move(stage_current));

        // ---------------------------------------------------
        // 3. STAGE 2: Bewege zu Ziel-Pose (MoveTo)
        // ---------------------------------------------------
        auto stage_move = std::make_unique<mtc::stages::MoveTo>("Move To Goal", sampling_planner);
        stage_move->setGroup("xarm7");
        stage_move->setIKFrame("link_eef"); // WICHTIG: Dein End-Effector Link Name

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "link_base";
        // Eine Position etwas weiter weg/oben (Beispielwerte!)
        target_pose.pose.position.x = 0.4;
        target_pose.pose.position.y = 0.2;
        target_pose.pose.position.z = 0.4;
        target_pose.pose.orientation.w = 1.0; // Ausrichtung nach unten oder wie benötigt

        stage_move->setGoal(target_pose);
        task_.add(std::move(stage_move));
    }

    void execute() {
        RCLCPP_INFO(get_logger(), "Starte Planung...");
        
        try {
            task_.init();
        } catch (mtc::InitStageException& e) {
            RCLCPP_ERROR_STREAM(get_logger(), "Init Fehler: " << e);
            return;
        }

        if (task_.plan(5)) { // Versuche 5 Lösungen zu finden
            RCLCPP_INFO(get_logger(), "Plan gefunden! Führe aus...");
            task_.introspection().publishSolution(*task_.solutions().front()); // Für RViz Visualisierung
            task_.execute(*task_.solutions().front());
        } else {
            RCLCPP_ERROR(get_logger(), "Kein Plan gefunden!");
        }
    }

private:
    mtc::Task task_;

    void createObstacle() {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "link_base";
        obj.id = "obstacle_box";
        
        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {0.1, 0.1, 0.4}; // 10x10cm breit, 40cm hoch

        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.3; // Steht zwischen Roboter und Ziel
        pose.position.y = 0.0;
        pose.position.z = 0.2;
        pose.orientation.w = 1.0;

        obj.primitives.push_back(prim);
        obj.primitive_poses.push_back(pose);
        obj.operation = obj.ADD;

        psi.applyCollisionObject(obj);
        // Kurz warten, bis MoveIt das Objekt registriert hat
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMTCNode>();
    
    // Wir nutzen einen separaten Thread für den Executor, damit Callbacks laufen
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    node->execute();

    rclcpp::shutdown();
    return 0;
}