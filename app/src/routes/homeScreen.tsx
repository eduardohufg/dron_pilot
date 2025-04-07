import ImageStream from "../components/imageStream";
import DroneCommands from "../components/droneCommands";

export default function Home() {
  return (
    <main className="w-screen h-screen overflow-hidden bg-neutral-900 flex flex-col">
      <div className="flex-shrink-0 py-4 px-6 text-center bg-neutral-800 shadow-md">
        <h1 className="text-blue-400 text-3xl sm:text-4xl font-bold tracking-wide">
          PX4 - Simulation
        </h1>
      </div>

      <div className="flex flex-1 flex-col sm:flex-row">
  <div className="basis-[66%] flex items-center justify-center p-4 overflow-hidden">
    <div className="rounded-xl shadow-lg">
      <ImageStream widthRatio={0.95} heightRatio={0.95} />
    </div>
  </div>

  <div className="basis-[34%] flex items-center justify-center p-4 overflow-y-auto">
    <div className="bg-neutral-800 rounded-xl shadow-lg p-6 w-full max-w-sm">
      <DroneCommands />
    </div>
  </div>
</div>
    </main>
  );
}
