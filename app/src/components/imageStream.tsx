import React, { useEffect, useRef, useState } from "react";
import { connectToImageStream } from "../services/api_credentials";

interface ImageStreamProps {
  widthRatio?: number;  
  heightRatio?: number; 
}

const ImageStream: React.FC<ImageStreamProps> = ({
  widthRatio = 0.66,
  heightRatio = 0.66,
}) => {
  const [connected, setConnected] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);
  const [imgSrc, setImgSrc] = useState<string>("");

  useEffect(() => {
    const ws = connectToImageStream({
      onOpen: () => setConnected(true),
      onMessage: (data) => {
        setImgSrc(`data:image/jpeg;base64,${data}`);
      },
      onClose: () => setConnected(false),
    });

    wsRef.current = ws;

    return () => {
      ws.close();
    };
  }, []);

  const maxWidth = `${widthRatio * 100}vw`;
  const maxHeight = `${heightRatio * 100}vh`;

  return (
    <div className="w-screen h-screen flex items-center justify-center bg-black overflow-hidden">
      <div className="text-center">
        <h2 className="text-white text-xl mb-4">
          {connected ? "Streaming" : "Conecting..."}
        </h2>
        {imgSrc ? (
          <img
            src={imgSrc}
            alt="Stream desde ROS2"
            style={{ maxWidth, maxHeight }}
            className="rounded shadow-lg object-contain"
          />
        ) : (
          <p className="text-gray-400">Esperando imagen...</p>
        )}
      </div>
    </div>
  );
};

export default ImageStream;
