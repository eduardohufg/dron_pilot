import ImageStream from "../components/imageStream";
import DroneCommands from "../components/droneCommands";

export default function Home() {
    return (
      <main className="w-screen h-screen overflow-hidden bg-black flex flex-col">
        {/* Cabecera fija arriba */}
        <div className="flex-shrink-0 p-4 text-center">
          <h1 className="text-blue-500 text-4xl font-bold">
            PX4 - ROS2 - Simulation
          </h1>
        </div>
  
        {/* Contenido principal en resto de la pantalla */}
        <div className="flex-1 grid grid-cols-1 sm:grid-cols-2">
          {/* Columna izq: Imagen */}
          <div className="flex items-center justify-center">
            {/* Ajusta widthRatio/heightRatio para que la imagen sea más chica */}
            <ImageStream widthRatio={0.5} heightRatio={0.5} />
          </div>
  
          {/* Columna der: Botones */}
          <div className="flex items-center justify-center">
            <DroneCommands />
          </div>
        </div>
      </main>
    );
  }