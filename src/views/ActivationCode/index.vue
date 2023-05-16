<template>
  <div class="activation-code">
    <h1 style="color: rgba(244,239,239,0.8); font-size: 30px; margin-bottom: 20px;">
      Welcome to Jack's Chatroom
    </h1>
    <div class="card">
      <h2>Enter Your Activation Code</h2>
      <div class="input-group">
        <input
          v-model="inputCode"
          type="password"
          placeholder="Enter activation code"
          maxlength="8"
        >
        <!--				<button @click="submitCode">Submit</button>-->
        <NButton @click="submitCode">
          Submit
        </NButton>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import { NAutoComplete, NButton, NInput, useDialog, useMessage } from 'naive-ui'

const inputCode = ref('')
const submitCode = () => {
  // Validate the activation code
  if (validateActivationCode(inputCode.value)) {
    // Save the activation code to a cookie
    const expiryDate = new Date()
    expiryDate.setMonth(expiryDate.getMonth() + 1)
    document.cookie = `activationCode=${inputCode.value}; expires=${expiryDate.toUTCString()}; path=/`
    this.$emit('valid-code')
    // refresh the page
    window.location.reload()
  } else {
    alert('Invalid activation code. Please try again.')
  }
}
const validateActivationCode = (code) => {
  return code === '110311'
}

</script>

<style scoped>
.activation-code {
	display: flex;
	flex-direction: column;
	align-items: center;
	justify-content: center;
	height: 100vh;
}

.card {
	display: flex;
	flex-direction: column;
	align-items: center;
	background-color: #ffffff;
	border-radius: 8px;
	padding: 2rem;
	box-shadow: 0 2px 8px rgba(0, 0, 0, 0.1);
}

.input-group {
	display: flex;
	flex-direction: column;
	align-items: center;
	margin-top: 1rem;
}

input {
	border: 1px solid #ebebeb;
	border-radius: 5px;
	padding: 0.5rem;
	font-size: 1rem;
	margin-bottom: 1rem;
}

button {
	background-color: white;
	border: 1px solid #409eff;
	border-radius: 5px;
	padding: 0.5rem 1rem;
	font-size: 1rem;
	color: #409eff;
	cursor: pointer;
}

button:hover {
	background-color: #409eff;
	color: white;
}
</style>
