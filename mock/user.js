
const tokens = {
  admin: {
    token: 'admin-token'
  },
  endUser: {
    token: 'endUser-token'
  }
}

const users = {
  'admin-token': {
    roles: ['admin'],
    introduction: 'I am a super administrator',
    avatar: '/favicon.ico',
    name: 'Super Admin'
  },
  'endUser-token': {
    roles: ['editor'],
    introduction: 'I am an end user',
    avatar: '/user.png',
    name: 'Normal End User'
  }
}

const passwords = {
  admin: '110311',
  endUser: '110311'
}

module.exports = [
  // user login
  {
    url: '/vue-element-admin/user/login',
    type: 'post',
    response: config => {
      const { username, password } = config.body
      let tokenKey = ''

      if (username === 'admin' && password === passwords.admin) {
        tokenKey = 'admin'
      } else if (username === 'endUser' && password === passwords.endUser) {
        tokenKey = 'endUser'
      }

      const token = tokens[tokenKey]

      // mock error
      if (!token) {
        return {
          code: 60204,
          message: 'Account and password are incorrect.'
        }
      }

      return {
        code: 20000,
        data: token
      }
    }

  },

  // get user info
  {
    url: '/vue-element-admin/user/info\.*',
    type: 'get',
    response: config => {
      const { token } = config.query
      const info = users[token]

      // mock error
      if (!info) {
        return {
          code: 50008,
          message: 'Login failed, unable to get user details.'
        }
      }

      return {
        code: 20000,
        data: info
      }
    }
  },

  // user logout
  {
    url: '/vue-element-admin/user/logout',
    type: 'post',
    response: _ => {
      return {
        code: 20000,
        data: 'success'
      }
    }
  }
]
