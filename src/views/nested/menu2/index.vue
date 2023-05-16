<template>
  <div>
    <h1>HILMAP</h1>
    <p>
      Speak into your microphone and get AI-generated responses in real time!
    </p>

    <div>
      <label for="chat-input">You:</label>
      <input id="chat-input" type="text">
    </div>

    <div>
      <label for="chat-output">AI:</label>
      <textarea id="chat-output" readonly />
    </div>

    <button id="record-button" onclick="recordAndTranscribe()">Record</button>
    <button id="stop-button">Stop</button>

    <label id="status-label" />
  </div>
</template>
