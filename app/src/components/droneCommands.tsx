import { useEffect, useRef, useState } from "react";
import { connectToMoveStream } from "../services/api_credentials";

/**
 * Estructura del objeto que enviamos cada 50 ms.
 */
type DroneControlState = {
    position: {
      x_pos: number;
      x_neg: number;
      y_pos: number;
      y_neg: number;
      z_pos: number;
      z_neg: number;
    };
    orientation: {
      yaw_pos: number;
      yaw_neg: number;
    };
    command: {
      init: number;
    };
  };
  
  export default function DroneCommands() {
    const [connected, setConnected] = useState(false);
  
    const [controlState, setControlState] = useState<DroneControlState>({
      position: { x_pos: 0, x_neg: 0, y_pos: 0, y_neg: 0, z_pos: 0, z_neg: 0 },
      orientation: { yaw_pos: 0, yaw_neg: 0 },
      command: { init: 0 },
    });
  
    const wsRef = useRef<WebSocket | null>(null);
    const controlStateRef = useRef(controlState);
  
    useEffect(() => {
      controlStateRef.current = controlState;
    }, [controlState]);
  
    useEffect(() => {
      const ws = connectToMoveStream({
        onOpen: () => setConnected(true),
        onClose: () => setConnected(false),
        onError: () => setConnected(false),
      });
      wsRef.current = ws;
  
      return () => {
        ws.close();
      };
    }, []);
  
    useEffect(() => {
      const intervalId = setInterval(() => {
        if (wsRef.current?.readyState === WebSocket.OPEN) {
          const dataToSend = JSON.stringify(controlStateRef.current);
          wsRef.current.send(dataToSend);
        }
      }, 50);
  
      return () => clearInterval(intervalId);
    }, []);
  
    const handleMouseDown = (
      category: "position" | "orientation" | "command",
      field: string
    ) => {
      setControlState((prev) => ({
        ...prev,
        [category]: {
          ...prev[category],
          [field]: 1,
        },
      }));
    };
  
    const handleMouseUp = (
      category: "position" | "orientation" | "command",
      field: string
    ) => {
      setControlState((prev) => ({
        ...prev,
        [category]: {
          ...prev[category],
          [field]: 0,
        },
      }));
    };
  
    const btnClass =
      "bg-blue-400 text-white font-bold py-2 px-4 rounded " +
      "hover:bg-blue-700 transition-colors focus:outline-none " +
      "focus:ring-2 focus:ring-blue-500";
  
    return (
      <div className="flex flex-col items-center space-y-6">
        <h2 className="text-xl text-white">
          {connected ? "Ready to send comamnds" : "Conecting..."}
        </h2>
  
        <div className="flex flex-col sm:flex-row items-center justify-center space-x-0 sm:space-x-8 space-y-4 sm:space-y-0">
          
          <div className="grid grid-rows-3 grid-cols-3 gap-2">
            <div></div>
            <button
              className={btnClass}
              onMouseDown={() => handleMouseDown("position", "x_pos")}
              onMouseUp={() => handleMouseUp("position", "x_pos")}
            >
              X+
            </button>
            <div></div>
  
            <button
              className={btnClass}
              onMouseDown={() => handleMouseDown("position", "y_neg")}
              onMouseUp={() => handleMouseUp("position", "y_neg")}
            >
              Y-
            </button>
           
            <div></div>
            <button
              className={btnClass}
              onMouseDown={() => handleMouseDown("position", "y_pos")}
              onMouseUp={() => handleMouseUp("position", "y_pos")}
            >
              Y+
            </button>
        
              <div></div>

            <button
              className={btnClass}
              onMouseDown={() => handleMouseDown("position", "x_neg")}
              onMouseUp={() => handleMouseUp("position", "x_neg")}
            >
              X-
            </button>
            
            <div></div>
          </div>
  
          <div className="flex flex-col items-center space-y-2">
            <button
              className={btnClass}
              onMouseDown={() => handleMouseDown("position", "z_pos")}
              onMouseUp={() => handleMouseUp("position", "z_pos")}
            >
              Z+
            </button>
            <button
              className={btnClass}
              onMouseDown={() => handleMouseDown("position", "z_neg")}
              onMouseUp={() => handleMouseUp("position", "z_neg")}
            >
              Z-
            </button>
          </div>
        </div>
  
        <div className="flex flex-row items-center space-x-4">
          <button
            className={btnClass}
            onMouseDown={() => handleMouseDown("orientation", "yaw_neg")}
            onMouseUp={() => handleMouseUp("orientation", "yaw_neg")}
          >
            Yaw-
          </button>
          <button
            className={btnClass}
            onMouseDown={() => handleMouseDown("orientation", "yaw_pos")}
            onMouseUp={() => handleMouseUp("orientation", "yaw_pos")}
          >
            Yaw+
          </button>
        </div>
  
        <button
          className={btnClass}
          onMouseDown={() => handleMouseDown("command", "init")}
          onMouseUp={() => handleMouseUp("command", "init")}
        >
          INIT
        </button>
      </div>
    );
  }