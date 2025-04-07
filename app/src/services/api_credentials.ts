import {WS_URL} from "../auth/constants"
import {WS_MOVE_URL} from "../auth/constants"

export type WSHandlers = {
  onMessage: (data: string) => void;
  onOpen?: () => void;
  onError?: (e: Event) => void;
  onClose?: () => void;
};

export type WSHandlersMove = {
    onMessage?: (data: any) => void;
    onOpen?: () => void;
    onClose?: () => void;
    onError?: (e: Event) => void;
  };


export function connectToImageStream({ onMessage, onOpen, onError, onClose }: WSHandlers): WebSocket {
  const ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    console.log("WebSocket conected");
    onOpen?.();
  };

  ws.onmessage = (event) => {
    onMessage(event.data);
  };

  ws.onerror = (e) => {
    console.error("WebSocket error", e);
    onError?.(e);
  };

  ws.onclose = () => {
    console.warn("🔌 WebSocket closed");
    onClose?.();
  };

  return ws;
}


export function connectToMoveStream({
    onMessage,
    onOpen,
    onClose,
    onError,
  }: WSHandlersMove): WebSocket {
    const ws = new WebSocket(WS_MOVE_URL);
  
    ws.onopen = () => {
      console.log("✅ WebSocket conectado a", WS_MOVE_URL);
      onOpen?.();
    };
  
    ws.onmessage = (event) => {
      onMessage?.(event.data);
    };
  
    ws.onerror = (e) => {
      console.error("❌ WebSocket error", e);
      onError?.(e);
    };
  
    ws.onclose = () => {
      console.warn("🔌 WebSocket cerrado", WS_MOVE_URL);
      onClose?.();
    };
  
    return ws;
  }
